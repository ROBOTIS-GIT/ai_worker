"""
Line-delimited JSON-RPC client for one `codex app-server --stdio` process.

Only the host's main loop calls request/next_message. A reader thread just
moves stdout lines into a queue, so the app-server never blocks on a full pipe
while the host is executing a robot tool.
"""

from collections import deque
import hashlib
import json
from pathlib import Path
import queue
import subprocess
import threading
import time


class TransportClosed(RuntimeError):
    """The app-server exited or closed its stdout."""


class RpcError(RuntimeError):
    """A JSON-RPC request returned an error object."""

    def __init__(self, method, error):
        super().__init__(f'{method} failed: {error}')
        self.method = method
        self.error = error


def redact(value):
    """Copy of value with long data: URLs replaced by their size and hash, for logs."""
    if isinstance(value, str):
        if value.startswith('data:') and len(value) > 256:
            header, _, body = value.partition(',')
            digest = hashlib.sha256(body.encode('utf-8')).hexdigest()[:16]
            return f'<{header} {len(body)} chars sha256:{digest}>'
        return value
    if isinstance(value, dict):
        return {key: redact(item) for key, item in value.items()}
    if isinstance(value, list):
        return [redact(item) for item in value]
    return value


class AppServerTransport:
    """stdio JSON-RPC to one app-server child process, logged to log_dir."""

    def __init__(self, argv, env, cwd, log_dir, popen=subprocess.Popen):
        log_dir = Path(log_dir)
        self._in_log = (log_dir / 'rpc_in.jsonl').open('a', buffering=1, encoding='utf-8')
        self._out_log = (log_dir / 'rpc_out.jsonl').open('a', buffering=1, encoding='utf-8')
        self._stderr = (log_dir / 'app_stderr.log').open('a', buffering=1, encoding='utf-8')
        self.process = popen(
            argv, env=env, cwd=str(cwd),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._stderr,
            text=True, encoding='utf-8', bufsize=1,
        )
        self._incoming = queue.Queue()
        self._stash = deque()
        self._next_id = 1
        self._send_lock = threading.Lock()
        self._closed = False
        self._reader = threading.Thread(target=self._read_stdout, daemon=True)
        self._reader.start()

    def _read_stdout(self):
        try:
            for line in self.process.stdout:
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    message = {'_invalid_line': line[:2000]}
                self._in_log.write(json.dumps(redact(message), ensure_ascii=False) + '\n')
                self._incoming.put(message)
        finally:
            self._incoming.put({'_closed': True})

    def _closed_reason(self):
        return f'app-server closed (exit code {self.process.poll()}); see app_stderr.log'

    def _take(self, timeout):
        """Next raw message from the reader, or None on timeout."""
        if self._closed:
            raise TransportClosed(self._closed_reason())
        try:
            message = self._incoming.get(timeout=timeout)
        except queue.Empty:
            return None
        if message.get('_closed'):
            self._closed = True
            raise TransportClosed(self._closed_reason())
        return message

    def _send(self, message):
        line = json.dumps(message, separators=(',', ':'))
        with self._send_lock:
            if self._closed or self.process.poll() is not None:
                raise TransportClosed(self._closed_reason())
            self._out_log.write(json.dumps(redact(message), ensure_ascii=False) + '\n')
            try:
                self.process.stdin.write(line + '\n')
                self.process.stdin.flush()
            except OSError as exc:
                self._closed = True
                raise TransportClosed(f'write to app-server failed: {exc}') from exc

    def next_message(self, timeout):
        """Next notification or server request in arrival order, or None on timeout."""
        if self._stash:
            return self._stash.popleft()
        return self._take(timeout)

    def request(self, method, params, timeout):
        """Send a request and wait for its response. Other messages keep their order."""
        identifier = self._next_id
        self._next_id += 1
        self._send({'jsonrpc': '2.0', 'id': identifier, 'method': method, 'params': params})
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'no response to {method} within {timeout} s')
            message = self._take(min(remaining, 1.0))
            if message is None:
                continue
            if message.get('id') == identifier and 'method' not in message:
                if 'error' in message:
                    raise RpcError(method, message['error'])
                return message.get('result')
            self._stash.append(message)

    def notify(self, method, params):
        self._send({'jsonrpc': '2.0', 'method': method, 'params': params})

    def reply(self, identifier, result):
        self._send({'jsonrpc': '2.0', 'id': identifier, 'result': result})

    def reply_error(self, identifier, code, message):
        self._send({'jsonrpc': '2.0', 'id': identifier,
                    'error': {'code': code, 'message': message}})

    def close(self, timeout=10.0):
        try:
            self.process.stdin.close()
        except OSError:
            pass
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=timeout)
        # The reader writes rpc_in.jsonl; let it finish before closing the logs.
        self._reader.join(timeout=timeout)
        self._closed = True
        for stream in (self.process.stdout, self._in_log, self._out_log, self._stderr):
            if stream is not None:
                stream.close()

