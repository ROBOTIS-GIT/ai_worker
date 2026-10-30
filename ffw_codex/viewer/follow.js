// Follow run files through the same origin that serves run_viewer.html.
// This works with Live Server over SSH: the browser never needs access to the
// robot's filesystem and no new HTTP service is started by this page.
(() => {
  const intervalMs = 1200;
  let watcher = null;

  function setStatus(message, error = false) {
    text('followStatus', message);
    $('followStatus').classList.toggle('bad', error);
  }

  async function fetchText(watch, path) {
    const url = new URL(path, watch.base);
    const response = await fetch(url, {cache: 'no-store'});
    if (response.status === 404) return null;
    if (!response.ok) throw new Error(`Could not read ${path} (HTTP ${response.status}).`);
    return response.text();
  }

  async function directoryNames(watch, path) {
    const html = await fetchText(watch, path);
    if (html === null) return null;
    const doc = new DOMParser().parseFromString(html, 'text/html');
    return new Set([...doc.querySelectorAll('a[href]')].map(link => {
      const url = new URL(link.getAttribute('href'), watch.base);
      return decodeURIComponent(url.pathname.split('/').pop());
    }));
  }

  function recordText(files, path, content) {
    if (content === null || content === files[path]) return false;
    // JSON files are written atomically, but a browser can still see a partial
    // response from a generic file server. Retry on the next poll.
    if (path.endsWith('.json')) {
      try { JSON.parse(content); } catch { return false; }
    }
    files[path] = content;
    return true;
  }

  async function poll(watch) {
    let changed = false;
    const files = {...watch.files};
    const run = await fetchText(watch, 'run.json');
    if (run === null) throw new Error(`Run ${watch.id} has no run.json yet. Check the run ID and Live Server path.`);
    changed = recordText(files, 'run.json', run) || changed;

    const history = await fetchText(watch, 'public/history.jsonl');
    if (history !== null) {
      // Ignore an incomplete last line while the writer is appending it.
      const end = history.lastIndexOf('\n');
      if (end >= 0) changed = recordText(files, 'public/history.jsonl', history.slice(0, end + 1)) || changed;
    }
    const histories = (files['public/history.jsonl'] || '').trim().split('\n').filter(Boolean).map(line => JSON.parse(line));

    // Host call indices start at 0; rollout filenames use index+1. A request
    // exists before its result, which lets us show its recorded reason promptly.
    let nextCall = watch.nextCall;
    const callNames = await directoryNames(watch, 'calls/');
    if (callNames) {
      for (const name of [...callNames].filter(name => /^\d+_request\.json$/.test(name)).sort()) {
        const key = `calls/${name}`;
        if (files[key]) continue;
        const content = await fetchText(watch, key);
        if (content === null || !recordText(files, key, content)) continue;
        changed = true;
        nextCall = Math.max(nextCall, Number(name.slice(0, 4)) + 1);
      }
    } else {
      // Fallback for static servers without directory listings.
      for (let i = nextCall; i < watch.nextCall + 100; i++) {
        const key = `calls/${String(i).padStart(4, '0')}_request.json`;
        const content = await fetchText(watch, key);
        if (content === null) break;
        if (!recordText(files, key, content) && !files[key]) break;
        changed = true;
        nextCall = i + 1;
      }
    }
    const rolloutNames = await directoryNames(watch, 'rollout/');
    for (const [key, content] of Object.entries(files)) {
      if (!/^calls\/\d+_request\.json$/.test(key)) continue;
      const request = JSON.parse(content);
      const resultKey = `rollout/${String(request.index + 1).padStart(4, '0')}_${request.tool}.json`;
      if (files[resultKey] || (rolloutNames && !rolloutNames.has(resultKey.split('/')[1]))) continue;
      if (!rolloutNames && !histories.some(item => item.progress?.call_index === request.index + 1)) continue;
      const result = await fetchText(watch, resultKey);
      if (result !== null) changed = recordText(files, resultKey, result) || changed;
    }
    for (const item of histories) {
      const number = item.observation?.observation_number;
      if (!Number.isInteger(number)) continue;
      const folder = `public/observations/${String(number).padStart(3, '0')}`;
      const key = `${folder}/observation.json`;
      if (files[key]) continue;
      const observation = await fetchText(watch, key);
      if (observation === null || !recordText(files, key, observation)) continue;
      changed = true;
      // Images are written before observation.json, so their URLs are ready.
      for (const camera of ['head', 'wrist_left', 'wrist_right']) {
        const imageKey = `${folder}/${camera}.jpg`;
        files[imageKey] = new URL(imageKey, watch.base).href;
      }
    }
    const rootNames = await directoryNames(watch, '');
    for (const key of ['host_result.json', 'result.json']) {
      if (files[key] || (rootNames && !rootNames.has(key))) continue;
      const content = await fetchText(watch, key);
      if (content !== null) changed = recordText(files, key, content) || changed;
    }
    if (watcher !== watch) return;
    if (changed) {
      load(files, {refresh: true, followLatest: watch.latest, watching: true});
      watch.files = files;
      watch.nextCall = nextCall;
      if (watch.latest && state.events.length) scrollHistory();
    }
    watch.nextCall = nextCall;
    const count = state.events.length;
    const observationCount = Object.keys(watch.files).filter(key => key.endsWith('/observation.json')).length;
    const finished = Boolean(watch.files['host_result.json']);
    setStatus(`${finished ? 'Run finished' : 'Watching'} ${watch.id} · ${count} calls · ${observationCount} observations · ${watch.latest ? 'latest' : 'paused on selected call'}`);
    if (!finished && watcher === watch) watch.timer = setTimeout(() => tick(watch), intervalMs);
  }

  async function tick(watch) {
    try { await poll(watch); }
    catch (error) {
      if (watcher !== watch) return;
      setStatus(error.message, true);
      watch.timer = setTimeout(() => tick(watch), intervalMs);
    }
  }

  function stopFollow() {
    if (watcher) clearTimeout(watcher.timer);
    watcher = null;
    $('stopFollow').hidden = true;
    $('followLatest').hidden = true;
    text('runBadge', 'RECORDED RUN');
    setStatus('Not following a run.');
  }
  window.stopRunFollow = stopFollow;

  window.pauseRunFollowLatest = () => {
    if (!watcher || !watcher.latest) return;
    watcher.latest = false;
    $('followLatest').textContent = 'Resume latest';
    setStatus(`Watching ${watcher.id} · paused on selected call`);
  };

  $('followRun').onclick = () => {
    if (!['http:', 'https:'].includes(location.protocol)) {
      setStatus('Open this page through Live Server to follow a robot run. Offline files can be opened manually.', true);
      return;
    }
    const id = $('followRunId').value.trim();
    if (!/^[A-Za-z0-9_-]+$/.test(id)) {
      setStatus('Enter one run folder name, such as 20260928_003947.', true);
      return;
    }
    stopFollow();
    const base = new URL(`runs/${encodeURIComponent(id)}/`, new URL('.', location.href));
    watcher = {id, base, files: {}, nextCall: 0, latest: true, timer: null};
    $('stopFollow').hidden = false;
    $('followLatest').hidden = false;
    $('followLatest').textContent = 'Following latest';
    setStatus(`Connecting to ${id}…`);
    tick(watcher);
  };
  $('followRunId').onkeydown = event => {
    if (event.key === 'Enter') {event.preventDefault();$('followRun').click()}
  };
  $('stopFollow').onclick = stopFollow;
  $('followLatest').onclick = () => {
    if (!watcher) return;
    watcher.latest = !watcher.latest;
    $('followLatest').textContent = watcher.latest ? 'Following latest' : 'Resume latest';
    if (watcher.latest && state.events.length) {
      stop();state.index = state.events.length - 1;render();scrollHistory();
    }
    setStatus(`Watching ${watcher.id} · ${watcher.latest ? 'latest' : 'paused on selected call'}`);
  };
})();
