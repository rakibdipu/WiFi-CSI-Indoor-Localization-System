// dashboard/app.js — Digital Twin 2D Room Renderer & WebSocket Telemetry Client

const canvas = document.getElementById('roomCanvas');
const ctx = canvas.getContext('2d');

// Chart.js Subcarrier Amplitude Spectrum Setup
const chartCtx = document.getElementById('csiChart').getContext('2d');
const csiChart = new Chart(chartCtx, {
    type: 'bar',
    data: {
        labels: Array.from({length: 28}, (_, i) => `SC ${i+1}`),
        datasets: [{
            label: 'Subcarrier Amplitude |H(k)|',
            data: Array(28).fill(15),
            backgroundColor: 'rgba(56, 189, 248, 0.65)',
            borderColor: 'rgba(56, 189, 248, 1)',
            borderWidth: 1.5,
            borderRadius: 4
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 80 },
        scales: {
            y: {
                min: 0,
                max: 35,
                grid: { color: 'rgba(255, 255, 255, 0.05)' },
                ticks: { color: '#94a3b8', font: { size: 10 } }
            },
            x: {
                grid: { display: false },
                ticks: { color: '#94a3b8', font: { size: 9 } }
            }
        },
        plugins: {
            legend: { display: false }
        }
    }
});

// UI Elements
const elPresence = document.getElementById('valPresence');
const elActivity = document.getElementById('valActivity');
const elCoords = document.getElementById('valCoords');
const elZone = document.getElementById('valZone');
const elDirection = document.getElementById('valDirection');
const elConfidence = document.getElementById('valConfidence');
const elRssi = document.getElementById('valRssi');
const elNoise = document.getElementById('valNoise');
const elPktRate = document.getElementById('valPktRate');
const connStatusBadge = document.getElementById('connStatusBadge');
const connStatusText = document.getElementById('connStatusText');

// Hardware Node Coordinates (in room meters)
const AP_POS = { x: 2.5, y: 0.2 };   // Phone Hotspot / Transmitter
const RX_POS = { x: 0.3, y: 5.7 };   // ESP32-S3 Receiver

let latestData = {
    x: 2.5,
    y: 3.0,
    trajectory: [],
    presence: true,
    activity: "Walking",
    zone: "Zone-A1",
    direction: "North",
    confidence: 0.85
};

// --------------------------------------------------------------------------
// 2D Room Digital Twin Rendering
// --------------------------------------------------------------------------
function drawRoom() {
    const W = canvas.width;
    const H = canvas.height;
    
    // Scale: 5.0m width -> W pixels, 6.0m height -> H pixels
    const scaleX = W / 5.0;
    const scaleY = H / 6.0;

    // Clear Canvas
    ctx.clearRect(0, 0, W, H);

    // 1. Draw Grid Lines (1 meter spacing)
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    for (let x = 0; x <= 5.0; x += 1.0) {
        ctx.beginPath();
        ctx.moveTo(x * scaleX, 0);
        ctx.lineTo(x * scaleX, H);
        ctx.stroke();
    }
    for (let y = 0; y <= 6.0; y += 1.0) {
        ctx.beginPath();
        ctx.moveTo(0, y * scaleY);
        ctx.lineTo(W, y * scaleY);
        ctx.stroke();
    }

    // 2. Draw Zone Labels
    ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.font = '12px system-ui';
    ctx.fillText('Zone A1', 10, 20);
    ctx.fillText('Zone A2', W / 2 + 10, 20);
    ctx.fillText('Zone B1', 10, H / 2 + 20);
    ctx.fillText('Zone B2', W / 2 + 10, H / 2 + 20);

    // 3. Draw RF Line-of-Sight (LoS) Ray (AP -> RX)
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(99, 102, 241, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(AP_POS.x * scaleX, AP_POS.y * scaleY);
    ctx.lineTo(RX_POS.x * scaleX, RX_POS.y * scaleY);
    ctx.stroke();
    ctx.setLineDash([]);

    // 4. Draw Transmitter AP (Phone)
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(AP_POS.x * scaleX, AP_POS.y * scaleY, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 10px system-ui';
    ctx.fillText('AP (Phone)', AP_POS.x * scaleX - 25, AP_POS.y * scaleY - 12);

    // 5. Draw Receiver (ESP32-S3)
    ctx.fillStyle = '#6366f1';
    ctx.beginPath();
    ctx.arc(RX_POS.x * scaleX, RX_POS.y * scaleY, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 10px system-ui';
    ctx.fillText('ESP32 RX', RX_POS.x * scaleX + 12, RX_POS.y * scaleY + 4);

    // 6. Draw Trajectory Trail
    if (latestData.trajectory && latestData.trajectory.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
        ctx.lineWidth = 2.5;
        latestData.trajectory.forEach((pt, i) => {
            const px = pt[0] * scaleX;
            const py = pt[1] * scaleY;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.stroke();

        // Trajectory points with fade
        latestData.trajectory.forEach((pt, i) => {
            const alpha = (i + 1) / latestData.trajectory.length;
            ctx.fillStyle = `rgba(6, 182, 212, ${alpha * 0.7})`;
            ctx.beginPath();
            ctx.arc(pt[0] * scaleX, pt[1] * scaleY, 3.5, 0, Math.PI * 2);
            ctx.fill();
        });
    }

    // 7. Draw Human Target Position 👤
    if (latestData.presence) {
        const hx = latestData.x * scaleX;
        const hy = latestData.y * scaleY;

        // Outer Pulsing Aura
        ctx.fillStyle = 'rgba(6, 182, 212, 0.18)';
        ctx.beginPath();
        ctx.arc(hx, hy, 22, 0, Math.PI * 2);
        ctx.fill();

        // Target Center Dot
        ctx.fillStyle = '#22d3ee';
        ctx.beginPath();
        ctx.arc(hx, hy, 9, 0, Math.PI * 2);
        ctx.fill();

        // Target Border
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Target Label & Coordinates
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 11px system-ui';
        ctx.fillText(`Target (${latestData.x}m, ${latestData.y}m)`, hx - 45, hy - 14);
    }

    requestAnimationFrame(drawRoom);
}

// Start room loop
requestAnimationFrame(drawRoom);

// --------------------------------------------------------------------------
// WebSocket Telemetry Connection
// --------------------------------------------------------------------------
function initWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/csi`;
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        connStatusBadge.className = "flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-semibold bg-emerald-500/20 text-emerald-400 border border-emerald-500/30";
        connStatusText.innerText = "Connected";
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            latestData = data;

            // Update UI Metrics
            elPresence.innerText = data.presence ? "Occupied" : "Empty Room";
            elPresence.className = data.presence ? "text-lg font-bold text-emerald-400 mt-1" : "text-lg font-bold text-slate-500 mt-1";
            
            elActivity.innerText = data.activity;
            elCoords.innerText = `(${data.x.toFixed(2)}, ${data.y.toFixed(2)}) m`;
            elZone.innerText = data.zone;
            elDirection.innerText = data.direction;
            elConfidence.innerText = `${Math.round(data.confidence * 100)}%`;
            elRssi.innerText = `${data.rssi} dBm`;
            elNoise.innerText = `${data.noise_floor} dBm`;
            elPktRate.innerText = `${data.packet_rate.toFixed(0)}`;

            // Update CSI Waveform Chart
            if (data.amplitudes && data.amplitudes.length > 0) {
                csiChart.data.datasets[0].data = data.amplitudes;
                csiChart.update('none');
            }
        } catch (e) {
            console.error("Error parsing WebSocket packet:", e);
        }
    };

    ws.onclose = () => {
        connStatusBadge.className = "flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-semibold bg-rose-500/20 text-rose-400 border border-rose-500/30";
        connStatusText.innerText = "Disconnected (Retrying...)";
        setTimeout(initWebSocket, 2000);
    };
}

initWebSocket();
