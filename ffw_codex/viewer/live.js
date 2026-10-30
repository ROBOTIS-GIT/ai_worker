// Uses Cyclo's existing web_video_server. No page server or third-party scripts.
(() => {
  const cameras = ['head', 'wrist_left', 'wrist_right'];
  const fields = ['topicHead', 'topicLeft', 'topicRight'];
  const presets = {
    robot: ['/zed/zed_node/rgb/image_rect_color', '/camera_left/camera_left/color/image_rect_raw', '/camera_right/camera_right/color/image_rect_raw'],
    gazebo: ['/head_camera/image', '/wrist_left_camera/image', '/wrist_right_camera/image'],
  };
  const snapshotNotice = $('viewNotice').textContent;
  let streams = [], monitor = null;

  function buildStreamUrl(base, topic, transport) {
    const url = new URL(base);
    if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash)
      throw new Error('Enter the video bridge URL as http://HOST:7085.');
    topic = topic.trim();
    if (!/^\/[A-Za-z0-9_/]+$/.test(topic))
      throw new Error('Camera topics must be ROS names starting with /.');
    if (transport === 'compressed' && topic.endsWith('/compressed')) topic = topic.slice(0, -11);
    // Cyclo's server expects literal slashes in the topic parameter.
    const type = transport === 'compressed' ? 'ros_compressed' : 'mjpeg';
    return `${url.href.replace(/\/+$/, '')}/stream?quality=50&type=${type}&default_transport=${transport}&topic=${topic}`;
  }
  window.buildCameraStreamUrl = buildStreamUrl;

  window.updateLiveLabels = () => {
    text('observationLabel', 'Camera feed');
    text('observation', 'LIVE SOURCE · decisions below use the selected record');
    text('viewNotice', 'The cameras show current streams; the decision and command below come from the selected saved run. They are not synchronized. The stream provides no frame timestamp, so frame freshness cannot be verified.');
    text('connectionMode', 'LOCAL · LIVE CAMERA');
  };

  function disconnect() {
    clearInterval(monitor); monitor = null;
    for (const s of streams) {
      s.img.onload = null; s.img.onerror = null;
      s.img.src = '';
      // Replace a streaming IMG to cancel its open multipart HTTP request.
      const replacement = document.createElement('img');
      replacement.alt = s.img.alt;
      s.img.replaceWith(replacement);
      s.el.classList.remove('expanded');
      s.empty.textContent = 'No saved image';
    }
    streams = [];
    $('disconnectLive').disabled = true;
    $('liveView').disabled = true;
    $('liveView').classList.remove('active');
    $('liveView').setAttribute('aria-pressed', 'false');
    text('connectionMode', 'LOCAL · OFFLINE');
    text('observationLabel', 'Displayed observation');
    text('viewNotice', snapshotNotice);
    text('liveStatus', 'Disconnected');
    if (state.view === 'live') state.view = 'before';
  }
  window.disconnectLiveCameras = disconnect;

  function refreshStatus() {
    let seen = 0, failed = 0;
    for (const s of streams) {
      // MJPEG load events vary by browser; naturalWidth also detects decoded data.
      if (!s.failed && s.img.naturalWidth > 0) {
        s.seen = true; s.empty.hidden = true;
      }
      if (s.seen && !s.failed) seen++;
      if (s.failed) failed++;
      s.stamp.textContent = s.failed ? 'STREAM ERROR' : s.seen ? 'LIVE SOURCE · frame received' : 'LIVE SOURCE · waiting for video';
    }
    text('liveStatus', `Frames received ${seen}/3${failed ? ` · stream errors ${failed}` : ''}`);
  }

  function connect(event) {
    event?.preventDefault();
    let urls;
    try {
      urls = fields.map(id => buildStreamUrl($('streamBase').value.trim(), $(id).value, $('streamTransport').value));
    } catch (error) { text('liveError', error.message); return; }
    disconnect(); stop(); text('liveError', '');
    if ($('lightbox').open) $('lightbox').close();
    $('largeImage').removeAttribute('src');
    state.view = 'live';
    cameras.forEach((camera, i) => {
      const el = document.querySelector(`[data-camera="${camera}"]`);
      const old = el.querySelector('img');
      old.onload = null; old.onerror = null;
      const img = document.createElement('img'); img.alt = `${camera} live video`;
      old.replaceWith(img);
      const stream = { el, img, empty: el.querySelector('.empty'), stamp: el.querySelector('.stamp'), seen: false, failed: false };
      streams.push(stream);
      stream.empty.textContent = 'Waiting for video'; stream.empty.hidden = false;
      img.onload = refreshStatus;
      img.onerror = () => {
        stream.failed = true;
        stream.empty.textContent = 'Video connection failed'; stream.empty.hidden = false;
        img.removeAttribute('src');
        text('liveError', 'Check the bridge URL, camera topics, and Raw/Compressed transport. If your browser requests local network access, allow it and reconnect.');
        refreshStatus();
      };
      img.src = `${urls[i]}&t=${Date.now()}`;
    });
    $('disconnectLive').disabled = false;
    $('liveView').disabled = false;
    refreshStatus();
    monitor = setInterval(refreshStatus, 500);
    render();
  }

  $('liveSettingsButton').onclick = () => { $('liveSettings').open = !$('liveSettings').open; };
  $('topicPreset').onchange = () => presets[$('topicPreset').value].forEach((topic, i) => { $(fields[i]).value = topic; });
  $('liveForm').onsubmit = connect;
  $('disconnectLive').onclick = () => { disconnect(); render(); };
  $('liveView').onclick = () => { state.view = 'live'; render(); };
  // Expand the existing live image rather than opening a second streaming request.
  document.querySelectorAll('.camera').forEach(el => el.addEventListener('click', event => {
    if (state.view !== 'live') return;
    event.stopImmediatePropagation();
    el.classList.toggle('expanded');
  }, true));
  document.addEventListener('keydown', event => {
    if (event.key === 'Escape') document.querySelectorAll('.camera.expanded').forEach(el => el.classList.remove('expanded'));
  });
  window.addEventListener('pagehide', disconnect);
})();
