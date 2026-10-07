#ifndef INDEX_HTML_H
#define INDEX_HTML_H

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM Vision Assistant</title>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #0f172a;
      --card-bg: rgba(30, 41, 59, 0.7);
      --accent: #6366f1;
      --accent-hover: #4f46e5;
      --text: #f8fafc;
      --text-muted: #94a3b8;
      --border: rgba(255, 255, 255, 0.1);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Inter', sans-serif; }
    body {
      background: #090d16;
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 2rem 1rem;
    }
    .header {
      text-align: center;
      margin-bottom: 2rem;
    }
    .header h1 {
      font-size: 2rem;
      font-weight: 700;
      background: linear-gradient(135deg, #a5b4fc, #6366f1);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      margin-bottom: 0.5rem;
    }
    .header p { color: var(--text-muted); font-size: 0.95rem; }
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 1.5rem;
      max-width: 900px;
      width: 100%;
    }
    @media (min-width: 768px) {
      .grid { grid-template-columns: 1.2fr 1fr; }
    }
    .card {
      background: var(--card-bg);
      backdrop-filter: blur(12px);
      border: 1fr solid var(--border);
      border-radius: 1rem;
      padding: 1.25rem;
      box-shadow: 0 10px 25px -5px rgba(0,0,0,0.3);
      display: flex;
      flex-direction: column;
    }
    .card-title {
      font-size: 1.1rem;
      font-weight: 600;
      margin-bottom: 1rem;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }
    .status-badge {
      font-size: 0.75rem;
      padding: 0.25rem 0.6rem;
      border-radius: 999px;
      font-weight: 600;
      background: #334155;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .status-badge.analyzing { background: #854d0e; color: #fef08a; }
    .status-badge.ready { background: #166534; color: #bbf7d0; }
    .status-badge.error { background: #991b1b; color: #fecaca; }

    .video-wrapper {
      position: relative;
      width: 100%;
      aspect-ratio: 4/3;
      background: #000;
      border-radius: 0.75rem;
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .video-wrapper img {
      width: 100%;
      height: 100%;
      object-fit: cover;
    }
    .btn {
      margin-top: 1rem;
      width: 100%;
      padding: 0.85rem;
      border: none;
      border-radius: 0.75rem;
      background: linear-gradient(135deg, #6366f1, #4f46e5);
      color: #fff;
      font-size: 1rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s ease;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 0.5rem;
    }
    .btn:hover:not(:disabled) {
      transform: translateY(-2px);
      box-shadow: 0 4px 15px rgba(99, 102, 241, 0.4);
    }
    .btn:disabled {
      opacity: 0.6;
      cursor: not-allowed;
    }
    .spinner {
      width: 18px;
      height: 18px;
      border: 2.5px solid rgba(255,255,255,0.3);
      border-top-color: #fff;
      border-radius: 50%;
      animation: spin 0.8s linear infinite;
      display: none;
    }
    @keyframes spin { to { transform: rotate(360deg); } }

    .response-box {
      flex: 1;
      background: rgba(15, 23, 42, 0.6);
      border: 1px solid var(--border);
      border-radius: 0.75rem;
      padding: 1rem;
      font-size: 0.95rem;
      line-height: 1.6;
      white-space: pre-wrap;
      overflow-y: auto;
      max-height: 350px;
      color: #cbd5e1;
    }
  </style>
</head>
<body>

  <div class="header">
    <h1>ESP32-CAM Vision Assistant</h1>
    <p>Live Video Stream & Gemini 2.0 Flash Vision Recognition</p>
  </div>

  <div class="grid">
    <!-- Stream Card -->
    <div class="card">
      <div class="card-title">
        <span>Live Feed</span>
        <span class="status-badge ready" id="streamStatus">LIVE</span>
      </div>
      <div class="video-wrapper">
        <img src="/stream" id="videoFeed" alt="Live Stream">
      </div>
      <button class="btn" id="btnAnalyze" onclick="triggerAnalyze()">
        <span class="spinner" id="btnSpinner"></span>
        <span id="btnText">✨ Analyze with Gemini</span>
      </button>
    </div>

    <!-- Result Card -->
    <div class="card">
      <div class="card-title">
        <span>Gemini AI Insights</span>
        <span class="status-badge" id="aiStatus">IDLE</span>
      </div>
      <div class="response-box" id="resultText">Click 'Analyze with Gemini' to take a frame snapshot and list detected objects.</div>
    </div>
  </div>

  <script>
    let isPolling = false;

    async function triggerAnalyze() {
      const btn = document.getElementById('btnAnalyze');
      const spinner = document.getElementById('btnSpinner');
      const btnText = document.getElementById('btnText');
      const aiStatus = document.getElementById('aiStatus');
      
      btn.disabled = true;
      spinner.style.display = 'inline-block';
      btnText.innerText = 'Analyzing Image...';
      aiStatus.className = 'status-badge analyzing';
      aiStatus.innerText = 'ANALYZING';

      try {
        const resp = await fetch('/analyze', { method: 'POST' });
        const data = await resp.json();
        if (!isPolling) {
          pollStatus();
        }
      } catch (err) {
        console.error('Trigger error:', err);
        resetButton();
      }
    }

    async function pollStatus() {
      isPolling = true;
      const interval = setInterval(async () => {
        try {
          const resp = await fetch('/result');
          const data = await resp.json();
          
          const aiStatus = document.getElementById('aiStatus');
          const resultText = document.getElementById('resultText');

          if (data.status === 'analyzing') {
            aiStatus.className = 'status-badge analyzing';
            aiStatus.innerText = 'ANALYZING';
          } else if (data.status === 'completed') {
            aiStatus.className = 'status-badge ready';
            aiStatus.innerText = 'READY';
            resultText.innerText = data.result;
            resetButton();
            clearInterval(interval);
            isPolling = false;
          } else if (data.status === 'error') {
            aiStatus.className = 'status-badge error';
            aiStatus.innerText = 'ERROR';
            resultText.innerText = data.result;
            resetButton();
            clearInterval(interval);
            isPolling = false;
          }
        } catch (err) {
          console.error('Poll error:', err);
        }
      }, 1500);
    }

    function resetButton() {
      const btn = document.getElementById('btnAnalyze');
      const spinner = document.getElementById('btnSpinner');
      const btnText = document.getElementById('btnText');
      btn.disabled = false;
      spinner.style.display = 'none';
      btnText.innerText = '✨ Analyze with Gemini';
    }
  </script>
</body>
</html>
)rawliteral";

#endif
