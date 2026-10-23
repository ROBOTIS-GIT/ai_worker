"""
Codex app-server host: one thread per episode, dynamic tools served by a Rollout.

The host owns the app-server's stdio, so the model can reach the robot only
through dynamic tools. Robot tools run one at a time in a worker thread; the
main loop keeps reading events, watching limits and handling shutdown while a
command executes. Ending an episode never cancels a command already sent.
"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time
import traceback

from transport import AppServerTransport, RpcError, TransportClosed, redact


DISABLED_FEATURES = (
    'browser_use',
    'browser_use_external',
    'computer_use',
    'apps',
    'image_generation',
    'multi_agent',
    'plugins',
)
# Items that show the model is working; they reset the idle timer.
WORK_ITEMS = frozenset({
    'commandExecution', 'fileChange', 'imageView', 'dynamicToolCall', 'mcpToolCall',
})
# codexErrorInfo variants after which the same thread may continue.
RETRYABLE_ERRORS = frozenset({
    'httpConnectionFailed',
    'responseStreamConnectionFailed',
    'responseStreamDisconnected',
    'serverOverloaded',
    'internalServerError',
    'rateLimitExceeded',
})
CONTINUE_TEXT = 'The episode is not finished. Continue from the latest observation. Next call: '


class UncertainExecution(RuntimeError):
    """Raised by a Rollout when it cannot tell whether a robot command was sent."""


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def append_jsonl(path, record):
    with Path(path).open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + '\n')


def failure(text):
    """Dynamic tool reply the model reads as a failed call."""
    return {'success': False, 'contentItems': [{'type': 'inputText', 'text': text}]}


def error_kind(error):
    """Name of the codexErrorInfo variant in a turn error, or None."""
    info = (error or {}).get('codexErrorInfo')
    if isinstance(info, dict):
        return next(iter(info), None)
    return info


def render_codex_config(run_dir, codex_home, profile):
    """CODEX_HOME/config.toml text: permission profile for one run"""
    def quote(value):
        return json.dumps(str(value))  # JSON string escaping is valid TOML
    return '\n'.join([
        f'default_permissions = {quote(profile)}',
        '',
        '[shell_environment_policy]',
        'inherit = "none"',
        '',
        '[shell_environment_policy.set]',
        'PATH = "/usr/local/bin:/usr/bin:/bin"',
        'HOME = "/root"',
        'LANG = "C.UTF-8"',
        '',
        '[tools]',
        'web_search = false',
        '',
        f'[permissions.{profile}.filesystem]',
        '":root" = "deny"',
        '":minimal" = "read"',
        f'{quote(Path(codex_home) / "packages")} = "read"',
        '',
        f'[permissions.{profile}.filesystem.{quote(Path(run_dir).resolve())}]',
        '"public" = "read"',
        '"agent" = "read"',
        '"agent/notes" = "write"',
        '"agent/scratch" = "write"',
        '',
        f'[permissions.{profile}.network]',
        'enabled = false',
        '',
    ])


class CodexConfigFile:
    """Writes CODEX_HOME/config.toml for one run and removes it afterwards.

    One CODEX_HOME serves one run at a time, so an existing file is never
    overwritten: it may belong to a run that is still active.
    """

    def __init__(self, codex_home, text):
        self.path = Path(codex_home) / 'config.toml'
        self.text = text
        self.written = False

    def write(self):
        if self.path.exists():
            raise FileExistsError(f'{self.path} exists; another run may be active')
        self.path.write_text(self.text, encoding='utf-8')
        self.path.chmod(0o600)
        self.written = True

    def remove(self):
        if self.written:
            self.path.unlink(missing_ok=True)
            self.written = False


@dataclass
class HostConfig:
    developer_instructions: str
    initial_prompt: str
    model: str = 'gpt-6-astra'
    effort: str = 'low'
    codex: str = '/usr/local/bin/codex'
    codex_home: Path = Path('/root/.codex_robot')
    home: str = '/root'
    permission_profile: str = 'robot_run'
    disabled_features: tuple = DISABLED_FEATURES
    request_timeout_s: float = 60.0
    poll_s: float = 2.0
    idle_timeout_s: float = 900.0
    finish_grace_s: float = 120.0
    interrupt_wait_s: float = 30.0
    max_continuations: int = 2
    network_retry_delays_s: tuple = (10.0,) * 20
    token_limit: int | None = None


@dataclass
class ToolCall:
    rpc_id: object
    thread_id: str
    turn_id: str
    call_id: str
    tool: str
    arguments: object


class CallLedger:
    """Dynamic tool calls by callId, so a redelivered call never runs twice.

    calls/NNNN_request.json is written before execution starts (status
    running); NNNN_result.json is written when the reply is known.
    """

    def __init__(self, calls_dir):
        self.dir = Path(calls_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._entries = {}

    @staticmethod
    def digest(tool, arguments):
        text = json.dumps([tool, arguments], sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(text.encode('utf-8')).hexdigest()

    def count(self):
        return len(self._entries)

    def lookup(self, call):
        """('new'|'pending'|'completed'|'conflict', reply or reason)."""
        entry = self._entries.get(call.call_id)
        if entry is None:
            return 'new', None
        if entry['digest'] != self.digest(call.tool, call.arguments):
            return 'conflict', 'Rejected: this callId was already used with different arguments.'
        if entry['status'] in ('queued', 'running'):
            return 'pending', None
        return 'completed', entry['reply']

    def register(self, call):
        self._entries[call.call_id] = {
            'index': len(self._entries), 'call': call, 'status': 'queued', 'reply': None,
            'digest': self.digest(call.tool, call.arguments), 'rpc_ids': [call.rpc_id],
        }

    def attach(self, call_id, rpc_id):
        """A redelivery of a pending call gets the same reply when it is ready."""
        self._entries[call_id]['rpc_ids'].append(rpc_id)

    def begin(self, call_id):
        entry = self._entries[call_id]
        call = entry['call']
        entry['status'] = 'running'
        write_json(self.dir / f'{entry["index"]:04d}_request.json', {
            'index': entry['index'], 'call_id': call.call_id, 'thread_id': call.thread_id,
            'turn_id': call.turn_id, 'tool': call.tool, 'arguments': call.arguments,
            'digest': entry['digest'], 'status': 'running', 'started_at': utc_now(),
        })

    def complete(self, call_id, reply, status):
        """Store the reply and return every rpc id that is waiting for it."""
        entry = self._entries[call_id]
        entry['status'] = status
        entry['reply'] = reply
        write_json(self.dir / f'{entry["index"]:04d}_result.json', {
            'index': entry['index'], 'call_id': call_id, 'status': status,
            'finished_at': utc_now(), 'reply': redact(reply),
        })
        waiting, entry['rpc_ids'] = entry['rpc_ids'], []
        return waiting


class CodexHost:
    """Runs one episode: start app-server, serve one thread until an end reason"""

    def __init__(self, cfg, rollout, run_dir, transport_factory=AppServerTransport):
        self.cfg = cfg
        self.rollout = rollout
        self.run_dir = Path(run_dir).resolve()
        self.agent_dir = self.run_dir / 'agent'
        self.transport_factory = transport_factory
        self.transport = None
        self.ledger = CallLedger(self.run_dir / 'calls')
        self.config_file = CodexConfigFile(cfg.codex_home, render_codex_config(
            self.run_dir, cfg.codex_home, cfg.permission_profile))
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='robot-tool')
        self.server_info = {}
        self.thread_id = None
        self.turn_id = None
        self.turn_active = False
        self.turns = 0
        self.continuations = 0
        self.network_continues = 0
        self.retry_at = None
        self.queue = deque()
        self.active = None
        self.last_activity = time.monotonic()
        self.finished_at = None
        self.interrupt_deadline = None
        self.end_reason = None
        self.end_detail = None
        self.token_usage = None
        self.errors = []
        self.started = time.monotonic()

    # Lifecycle -----------------------------------------------------------

    def run(self):
        """Run the episode and return the host outcome; shutdown always runs."""
        try:
            self._start()
            self._start_turn(self.cfg.initial_prompt)
            try:
                self._loop()
            except KeyboardInterrupt:
                # First Ctrl+C: accept no new actions, let a running tool finish.
                self._end('interrupted_by_user')
                self._loop()
        except TransportClosed as exc:
            self._end('transport_closed', str(exc))
        except Exception as exc:
            self.errors.append({'at': utc_now(), 'traceback': traceback.format_exc()})
            self._end('host_error', f'{type(exc).__name__}: {exc}')
        finally:
            outcome = self._shutdown()
        return outcome

    def _start(self):
        if not self.agent_dir.is_dir():
            raise FileNotFoundError(f'agent workspace missing: {self.agent_dir}')
        self.config_file.write()
        argv = [self.cfg.codex, 'app-server', '--stdio', '--strict-config']
        for feature in self.cfg.disabled_features:
            argv += ['--disable', feature]
        # Only these variables reach app-server; API keys stay in CODEX_HOME auth.
        env = {
            'PATH': '/usr/local/bin:/usr/bin:/bin',
            'HOME': self.cfg.home,
            'LANG': 'C.UTF-8',
            'CODEX_HOME': str(self.cfg.codex_home),
        }
        self.transport = self.transport_factory(argv, env, self.run_dir, self.run_dir)
        self.server_info = self.transport.request('initialize', {
            'clientInfo': {'name': 'poc-codex-host', 'version': '1'},
            'capabilities': {'experimentalApi': True},
        }, self.cfg.request_timeout_s) or {}
        self.transport.notify('initialized', {})
        specs = self.rollout.tool_specs()
        thread = self.transport.request('thread/start', {
            'cwd': str(self.agent_dir),
            'model': self.cfg.model,
            'config': {'model_reasoning_effort': self.cfg.effort},
            'permissions': self.cfg.permission_profile,
            'approvalPolicy': 'never',
            'developerInstructions': self.cfg.developer_instructions,
            'dynamicTools': specs,
            'ephemeral': False,
        }, self.cfg.request_timeout_s)
        self.thread_id = thread['thread']['id']
        if thread.get('model') != self.cfg.model or thread.get('reasoningEffort') != self.cfg.effort:
            raise RuntimeError(
                f'thread uses {thread.get("model")}/{thread.get("reasoningEffort")}, '
                f'expected {self.cfg.model}/{self.cfg.effort}')

        def sha(text):
            return hashlib.sha256(text.encode('utf-8')).hexdigest()
        write_json(self.run_dir / 'host.json', {
            'started_at': utc_now(), 'argv': argv, 'server': self.server_info.get('userAgent'),
            'thread_id': self.thread_id, 'model': self.cfg.model, 'effort': self.cfg.effort,
            'permission_profile': self.cfg.permission_profile,
            'tools': [spec.get('name') for spec in specs],
            'developer_instructions_sha256': sha(self.cfg.developer_instructions),
            'initial_prompt_sha256': sha(self.cfg.initial_prompt),
            'codex_config_sha256': sha(self.config_file.text),
        })

    def _start_turn(self, text):
        result = self.transport.request('turn/start', {
            'threadId': self.thread_id,
            'input': [{'type': 'text', 'text': text}],
        }, self.cfg.request_timeout_s)
        self.turn_id = result['turn']['id']
        self.turn_active = True
        self.turns += 1
        self.interrupt_deadline = None
        self.last_activity = time.monotonic()
        append_jsonl(self.run_dir / 'turns.jsonl', {
            'event': 'started', 'index': self.turns, 'turn_id': self.turn_id,
            'at': utc_now(), 'text': text,
        })

    def _finished(self):
        return (self.end_reason is not None and self.active is None
                and not self.queue and not self.turn_active)

    def _loop(self):
        while not self._finished():
            self._poll_worker()
            self.rollout.tick()
            self._check_limits()
            if self._finished():
                break
            wait_s = 0.1 if self.active is not None else self.cfg.poll_s
            message = self.transport.next_message(wait_s)
            if message is not None:
                self._dispatch(message)

    def _end(self, reason, detail=None):
        """Record the first end reason, block new actions and drop queued calls"""
        if self.end_reason is not None:
            return
        self.end_reason, self.end_detail = reason, detail
        self.retry_at = None
        if reason != 'rollout_finished':
            self.rollout.halt(reason)
        while self.queue:
            self._reject(self.queue.popleft(), f'Not executed: the episode ended ({reason}).')

    def _check_limits(self):
        now = time.monotonic()
        if self.finished_at is None and self.rollout.finished():
            self.finished_at = now
        if self.end_reason is None:
            total = ((self.token_usage or {}).get('total') or {}).get('totalTokens', 0)
            if self.finished_at is not None and (
                    not self.turn_active or now - self.finished_at >= self.cfg.finish_grace_s):
                # The model gets finish_grace_s to read the final result and stop.
                self._end('rollout_finished')
            elif (self.turn_active and self.active is None
                  and now - self.last_activity >= self.cfg.idle_timeout_s):
                self._end('idle_timeout')
            elif self.cfg.token_limit and total >= self.cfg.token_limit:
                self._end('token_limit', total)
        if (self.end_reason is None and not self.turn_active
                and self.retry_at is not None and now >= self.retry_at):
            self.retry_at = None
            self._start_turn(self._continue_text())
        if self.end_reason is not None and self.turn_active and self.active is None:
            self._interrupt(now)

    def _interrupt(self, now):
        if self.interrupt_deadline is None:
            self.interrupt_deadline = now + self.cfg.interrupt_wait_s
            try:
                self.transport.request('turn/interrupt', {
                    'threadId': self.thread_id, 'turnId': self.turn_id,
                }, self.cfg.request_timeout_s)
            except (RpcError, TimeoutError) as exc:
                self.errors.append({'at': utc_now(), 'interrupt': str(exc)})
        elif now >= self.interrupt_deadline:
            self.errors.append({'at': utc_now(), 'interrupt': 'no turn/completed after interrupt'})
            self.turn_active = False

    def _continue_text(self):
        return CONTINUE_TEXT + json.dumps(self.rollout.next_call(), ensure_ascii=False)

    # Events --------------------------------------------------------------

    def _dispatch(self, message):
        if '_invalid_line' in message:
            self.errors.append({'at': utc_now(), 'invalid_line': message['_invalid_line'][:300]})
            return
        method = message.get('method')
        params = message.get('params') or {}
        if 'id' in message and method:
            if method == 'item/tool/call':
                self._on_tool_call(message['id'], params)
            else:
                # approvalPolicy never: approvals and other server requests are unexpected.
                self.errors.append({'at': utc_now(), 'unexpected_server_request': method})
                self.transport.reply_error(message['id'], -32601, f'{method} is not supported')
            return
        if params.get('threadId') not in (None, self.thread_id):
            return
        if method == 'thread/tokenUsage/updated':
            self.token_usage = params.get('tokenUsage')
            write_json(self.run_dir / 'token_usage.json',
                       {'at': utc_now(), 'thread_id': self.thread_id, 'usage': self.token_usage})
        elif method in ('item/started', 'item/completed'):
            self._on_item(method, params.get('item') or {})
        elif method == 'error':
            self.errors.append({'at': utc_now(), 'error': params.get('error'),
                                'will_retry': params.get('willRetry')})
        elif method == 'turn/completed':
            self._on_turn_completed(params.get('turn') or {})

    def _on_item(self, method, item):
        kind = item.get('type')
        if kind in WORK_ITEMS or (kind == 'agentMessage' and method == 'item/completed'):
            append_jsonl(self.run_dir / 'agent_events.jsonl',
                         {'at': utc_now(), 'method': method, 'item': redact(item)})
        if kind in WORK_ITEMS:
            self.last_activity = time.monotonic()

    def _on_turn_completed(self, turn):
        if turn.get('id') != self.turn_id:
            return
        self.turn_active = False
        status, error = turn.get('status'), turn.get('error')
        append_jsonl(self.run_dir / 'turns.jsonl', {
            'event': 'completed', 'index': self.turns, 'turn_id': self.turn_id,
            'at': utc_now(), 'status': status, 'error': error,
        })
        if self.end_reason is not None:
            return
        if self.rollout.finished():
            self._end('rollout_finished')
        elif status == 'completed':
            if self.continuations >= self.cfg.max_continuations:
                self._end('model_ended')
            else:
                self.continuations += 1
                self._start_turn(self._continue_text())
        elif (status == 'failed' and error_kind(error) in RETRYABLE_ERRORS
              and self.network_continues < len(self.cfg.network_retry_delays_s)):
            # Same thread, no action replay: the model decides what to do next.
            delay = self.cfg.network_retry_delays_s[self.network_continues]
            self.network_continues += 1
            self.retry_at = time.monotonic() + delay
        else:
            self._end(f'turn_{status}', error)

    # Tool calls ----------------------------------------------------------

    def _on_tool_call(self, rpc_id, params):
        call = ToolCall(rpc_id, params.get('threadId'), params.get('turnId'),
                        params.get('callId'), params.get('tool'), params.get('arguments'))
        if call.thread_id != self.thread_id or not call.call_id:
            self.transport.reply(rpc_id, failure('Rejected: unknown thread or missing callId.'))
            return
        self.last_activity = time.monotonic()
        state, payload = self.ledger.lookup(call)
        if state == 'completed':
            self.transport.reply(rpc_id, payload)
        elif state == 'pending':
            self.ledger.attach(call.call_id, rpc_id)
        elif state == 'conflict':
            self.transport.reply(rpc_id, failure(payload))
        elif self.end_reason is not None:
            self._reject(call, f'Not executed: the episode ended ({self.end_reason}).')
        else:
            self.ledger.register(call)
            self.queue.append(call)
            self._start_next()

    def _start_next(self):
        if self.active is not None or not self.queue:
            return
        call = self.queue.popleft()
        self.ledger.begin(call.call_id)  # recorded as running before anything executes
        future = self.executor.submit(self.rollout.handle_tool_call, call.tool, call.arguments)
        self.active = (call, future)

    def _poll_worker(self):
        if self.active is None or not self.active[1].done():
            return
        call, future = self.active
        self.active = None
        status = 'completed'
        try:
            reply = future.result()
            if not (isinstance(reply, dict) and isinstance(reply.get('success'), bool)
                    and isinstance(reply.get('contentItems'), list)):
                raise UncertainExecution(f'rollout returned {type(reply).__name__}, not a reply')
        except Exception as exc:
            # Includes UncertainExecution: a command may or may not have been sent.
            status = 'uncertain'
            reply = failure(f'Execution state of {call.tool} is uncertain; movement is '
                            'blocked for this episode.')
            self._end('uncertain_execution', f'{type(exc).__name__}: {exc}')
        for rpc_id in self.ledger.complete(call.call_id, reply, status):
            self.transport.reply(rpc_id, reply)
        self.last_activity = time.monotonic()
        self._start_next()

    def _reject(self, call, text):
        """Reply to a call that never executed and record it as rejected."""
        reply = failure(text)
        if self.ledger.lookup(call)[0] == 'new':
            self.ledger.register(call)
        for rpc_id in self.ledger.complete(call.call_id, reply, 'rejected'):
            self.transport.reply(rpc_id, reply)

    # Shutdown ------------------------------------------------------------

    def _shutdown(self):
        # A running robot tool is waited for; it cannot be cancelled.
        self.executor.shutdown(wait=True)
        if self.active is not None:
            try:
                self._poll_worker()
            except TransportClosed:
                pass
        if self.transport is not None:
            if self.turn_active:
                try:
                    self.transport.request('turn/interrupt', {
                        'threadId': self.thread_id, 'turnId': self.turn_id,
                    }, 5.0)
                except Exception as exc:
                    self.errors.append({'at': utc_now(), 'shutdown_interrupt': str(exc)})
            self.transport.close()
        self.config_file.remove()
        try:
            rollout_outcome = self.rollout.outcome()
        except Exception as exc:
            rollout_outcome = {'error': f'{type(exc).__name__}: {exc}'}
        outcome = {
            'finished_at': utc_now(),
            'elapsed_s': round(time.monotonic() - self.started, 1),
            'end_reason': self.end_reason,
            'end_detail': self.end_detail,
            'thread_id': self.thread_id,
            'server': self.server_info.get('userAgent'),
            'turns': self.turns,
            'continuations': self.continuations,
            'network_continues': self.network_continues,
            'tool_calls': self.ledger.count(),
            'token_usage': self.token_usage,
            'session_log': self._copy_session_log(),
            'errors': self.errors[-20:],
            'rollout': rollout_outcome,
        }
        write_json(self.run_dir / 'host_result.json', outcome)
        return outcome

    def _copy_session_log(self):
        """Failed Codex tool calls may appear only in this log"""
        sessions = Path(self.cfg.codex_home) / 'sessions'
        if not self.thread_id or not sessions.is_dir():
            return None
        matches = sorted(sessions.rglob(f'*{self.thread_id}*.jsonl'))
        if not matches:
            return None
        target = self.run_dir / 'codex_session.jsonl'
        shutil.copyfile(matches[-1], target)
        return str(target)
