const { ipcRenderer } = require('electron');

// State Management
let isConnected = false;
let offsets = { F: 0.0, Y: 0.0, P: 0.0 };

// UI Elements
const ui = {
    btnConnect: document.getElementById('btnConnect'),
    status: document.getElementById('status'),
    comPort: document.getElementById('comPort'),
    btnCalibrate: document.getElementById('btnCalibrate'),
    btnResetCal: document.getElementById('btnResetCal'),
    chkSafety: document.getElementById('chkSafety'),
    sliderB: document.getElementById('sliderB'),
    numB: document.getElementById('numB'),
    servos: {
        F: { slider: document.getElementById('sliderF'), num: document.getElementById('numF'), lblRel: document.getElementById('lblFRel'), lblAbs: document.getElementById('lblFAbs') },
        Y: { slider: document.getElementById('sliderY'), num: document.getElementById('numY'), lblRel: document.getElementById('lblYRel'), lblAbs: document.getElementById('lblYAbs') },
        P: { slider: document.getElementById('sliderP'), num: document.getElementById('numP'), lblRel: document.getElementById('lblPRel'), lblAbs: document.getElementById('lblPAbs') }
    }
};

// --- Connection Logic ---
ui.btnConnect.addEventListener('click', () => {
    if (!isConnected) ipcRenderer.send('connect-serial', ui.comPort.value);
    else window.location.reload(); 
});

ipcRenderer.on('serial-status', (event, status) => {
    ui.status.innerText = status;
    if (status.includes('Connected')) {
        isConnected = true;
        ui.status.style.color = '#00bfa5';
        ui.btnConnect.innerText = 'Disconnect';
        ui.btnConnect.style.background = '#ff4444';
        enableControls(true);
        // NO INITIAL DATA SPAM. The app stays dead silent upon connection.
    } else {
        ui.status.style.color = '#ff4444';
    }
});

function sendCommand(cmd) {
    if (isConnected) {
        console.log("App Fired Command:", cmd); // Press Ctrl+Shift+I in the app to see this
        ipcRenderer.send('send-command', cmd);
    }
}

// --- Servo Logic ---
function updateServo(axis, val) {
    let value = Math.max(0, Math.min(180, parseFloat(val))); 
    
    ui.servos[axis].slider.value = value;
    ui.servos[axis].num.value = value.toFixed(1);
    
    ui.servos[axis].lblAbs.innerText = value.toFixed(1);
    let rel = value - offsets[axis];
    ui.servos[axis].lblRel.innerText = (rel >= 0 ? "+" : "") + rel.toFixed(1);

    sendCommand(`${axis}:${value.toFixed(1)}`);
}

// THE MAGIC FIX: Using 'change' instead of 'input'
// Commands will ONLY send when you let go of the mouse button.
['F', 'Y', 'P'].forEach(axis => {
    ui.servos[axis].slider.addEventListener('change', (e) => updateServo(axis, e.target.value));
    ui.servos[axis].num.addEventListener('change', (e) => updateServo(axis, e.target.value));
});

// --- Calibration Logic ---
ui.btnCalibrate.addEventListener('click', () => {
    ['F', 'Y', 'P'].forEach(axis => {
        offsets[axis] = parseFloat(ui.servos[axis].slider.value);
        updateServo(axis, offsets[axis]); 
    });
    sendCommand('CALIBRATE:1');
});

ui.btnResetCal.addEventListener('click', () => {
    ['F', 'Y', 'P'].forEach(axis => {
        let currentAbs = parseFloat(ui.servos[axis].slider.value);
        offsets[axis] = 0.0;
        updateServo(axis, currentAbs); 
    });
});

// --- BLDC Safety Logic ---
function updateBldc(val) {
    let speed = parseFloat(val);
    
    if (!ui.chkSafety.checked && speed > 50.0) {
        speed = 50.0;
    }
    
    speed = Math.max(0, Math.min(100, speed)); 
    
    ui.sliderB.value = speed;
    ui.numB.value = speed.toFixed(1);
    
    sendCommand(`B:${speed.toFixed(1)}`);
}

// BLDC also restricted to release-to-fire
ui.sliderB.addEventListener('change', (e) => updateBldc(e.target.value));
ui.numB.addEventListener('change', (e) => updateBldc(e.target.value));
ui.chkSafety.addEventListener('change', () => updateBldc(ui.sliderB.value)); 

// --- UI Helper ---
function enableControls(state) {
    ui.btnCalibrate.disabled = !state;
    ui.btnResetCal.disabled = !state;
    ui.chkSafety.disabled = !state;
    ui.sliderB.disabled = !state;
    ui.numB.disabled = !state;
    ['F', 'Y', 'P'].forEach(axis => {
        ui.servos[axis].slider.disabled = !state;
        ui.servos[axis].num.disabled = !state;
    });
}