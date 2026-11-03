const { app, BrowserWindow, ipcMain } = require('electron');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

let port;
let mainWindow;

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 800,
        height: 700,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false // For local hardware dev simplicity
        }
    });
    mainWindow.loadFile('index.html');
}

app.whenReady().then(createWindow);

// Handle Serial Connection from UI
// Add this flag near the top of your file with your variables
let isConnecting = false;

// --- Replace the connect-serial block ---
ipcMain.on('connect-serial', (event, comPort) => {
    // 1. The Bouncer: Ignore double-clicks or overlapping connection attempts
    if (isConnecting) {
        console.log("Ignored overlapping connect attempt. Please wait...");
        return;
    }
    isConnecting = true;

    const connectToPort = () => {
        port = new SerialPort({ 
            path: comPort, 
            baudRate: 9600 
        }, (err) => {
            isConnecting = false; // Release the lock
            
            if (err) {
                console.log(`Connection Error: Opening ${comPort}:`, err.message);
                event.reply('serial-status', `Error: ${err.message}`);
                port = null; // Wipe the zombie variable
            } else {
                console.log(`Successfully opened ${comPort}`);
                event.reply('serial-status', `Connected to ${comPort}`);
            }
        });

        port.on('close', () => {
            console.log('Port physically closed.');
            port = null; // Ensure the variable is fully dead
            if (mainWindow) mainWindow.webContents.send('serial-status', 'Disconnected');
        });
        
        port.on('error', (err) => {
            console.log('Port Error:', err.message);
        });
    };

    // 2. Safely close any existing port before swapping
    if (port && port.isOpen) {
        console.log(`Closing existing connection...`);
        let oldPort = port;
        port = null; // Immediately sever the app's connection to the port
        
        oldPort.close((err) => {
            // Wait 1 full second for Windows to clear the RFCOMM handles
            setTimeout(connectToPort, 1000); 
        });
    } else {
        connectToPort();
    }
});

// --- Replace the send-command block ---
ipcMain.on('send-command', (event, command) => {
    // Drop all slider traffic if we are in the middle of connecting or swapping ports!
    if (isConnecting) return; 
    
    if (port && port.isOpen) {
        port.write(command + '\r\n', 'utf8', (err) => {
            if (err) {
                console.log('Write Error:', err.message);
            }
            // (Removed the console.log("Sent") here to keep your terminal clean)
        });
    } else {
        console.log('Failed: Port is closed.');
    }
});