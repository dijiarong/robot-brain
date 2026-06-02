"""Self-contained dashboard served by the FastAPI app."""

DASHBOARD_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Brain Console</title>
  <style>
    :root { color-scheme: dark; font-family: Inter, system-ui, sans-serif; background: #0b1220; color: #dce7f7; }
    body { margin: 0; padding: 28px; }
    main { max-width: 1120px; margin: auto; }
    header, section { background: #111c2e; border: 1px solid #24344f; border-radius: 14px; padding: 18px; margin-bottom: 16px; }
    h1, h2 { margin: 0 0 12px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; }
    .metric { background: #17253b; border-radius: 10px; padding: 12px; }
    .label { color: #90a5c5; font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }
    .value { font-size: 22px; margin-top: 6px; }
    form, .actions { display: flex; flex-wrap: wrap; gap: 10px; }
    input, button { border-radius: 8px; border: 1px solid #33496b; padding: 10px 12px; background: #17253b; color: #e9f2ff; }
    input { min-width: 260px; flex: 1; }
    button { cursor: pointer; }
    button:hover { background: #223553; }
    .danger { background: #632d3d; border-color: #a54559; }
    table { width: 100%; border-collapse: collapse; font-size: 14px; }
    th, td { text-align: left; padding: 9px 6px; border-bottom: 1px solid #24344f; }
    th { color: #90a5c5; }
    #connection { color: #8bc7ff; }
  </style>
</head>
<body>
<main>
  <header>
    <h1>Robot Brain Console</h1>
    <div id="connection">connecting...</div>
  </header>
  <section class="grid">
    <div class="metric"><div class="label">service</div><div class="value" id="service">-</div></div>
    <div class="metric"><div class="label">battery</div><div class="value" id="battery">-</div></div>
    <div class="metric"><div class="label">estop</div><div class="value" id="estop">-</div></div>
    <div class="metric"><div class="label">position</div><div class="value" id="position">-</div></div>
  </section>
  <section>
    <h2>Control</h2>
    <form id="task-form">
      <input id="objective" name="objective" placeholder="Enter task objective" required>
      <input id="priority" name="priority" type="number" value="0" aria-label="Priority">
      <button type="submit">Queue task</button>
    </form>
    <div class="actions" style="margin-top: 10px">
      <button id="warning">Send sample warning</button>
      <button id="stop" class="danger">Emergency stop</button>
      <button id="reset">Reset estop</button>
    </div>
  </section>
  <section>
    <h2>Tasks</h2>
    <table>
      <thead><tr><th>Status</th><th>Priority</th><th>Objective</th><th>Attempts</th></tr></thead>
      <tbody id="tasks"></tbody>
    </table>
  </section>
</main>
<script>
const request = (url, options = {}) => fetch(url, {headers: {'Content-Type': 'application/json'}, ...options});
async function refresh() {
  const response = await fetch('/api/status');
  render(await response.json());
}
function render(status) {
  const world = status.world;
  document.querySelector('#service').textContent = status.service.running ? 'running' : 'stopped';
  document.querySelector('#battery').textContent = `${world.battery_level.toFixed(1)}%`;
  document.querySelector('#estop').textContent = world.estop_active ? 'ACTIVE' : 'clear';
  document.querySelector('#position').textContent = `${world.position.x.toFixed(1)}, ${world.position.y.toFixed(1)}`;
  document.querySelector('#tasks').innerHTML = status.tasks.map(task =>
    `<tr><td>${task.status}</td><td>${task.priority}</td><td>${task.objective}</td><td>${task.attempts}/${task.max_attempts}</td></tr>`
  ).join('');
}
document.querySelector('#task-form').addEventListener('submit', async event => {
  event.preventDefault();
  await request('/api/tasks', {method: 'POST', body: JSON.stringify({
    objective: document.querySelector('#objective').value,
    priority: Number(document.querySelector('#priority').value)
  })});
  document.querySelector('#objective').value = '';
  await refresh();
});
document.querySelector('#warning').addEventListener('click', async () => {
  await request('/api/events', {method: 'POST', body: JSON.stringify({type: 'warning', message: 'sample warning from dashboard'})});
  await refresh();
});
document.querySelector('#stop').addEventListener('click', async () => {
  await request('/api/events', {method: 'POST', body: JSON.stringify({type: 'interrupt', message: 'dashboard emergency stop'})});
  await refresh();
});
document.querySelector('#reset').addEventListener('click', async () => {
  await request('/api/estop/reset', {method: 'POST'});
  await refresh();
});
const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
socket.addEventListener('open', () => document.querySelector('#connection').textContent = 'live updates connected');
socket.addEventListener('message', event => {
  const message = JSON.parse(event.data);
  if (message.type === 'status') render(message.status);
  else refresh();
});
socket.addEventListener('close', () => document.querySelector('#connection').textContent = 'live updates disconnected');
refresh();
</script>
</body>
</html>
"""
