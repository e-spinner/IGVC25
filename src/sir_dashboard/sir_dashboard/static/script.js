

// Global Variables
// ------------------------------------------------

// init socketio
const socket = io()


// Helpers
// -----------------------------------------------



// Socketio Event Handlers
// -----------------------------------------------
socket.on('connect', () => {
    console.log('Connected to server');
    document.getElementById('connection-indicator').className = 'status-indicator status-connected';
    document.getElementById('connection-text').textContent = 'Connected to Robot';
});

socket.on('disconnect', () => {
    console.log('Disconnected from server');
    document.getElementById('connection-indicator').className = 'status-indicator status-disconnected';
    document.getElementById('connection-text').textContent = 'Disconnected - Reconnecting...';
});

socket.on('map_update', (data) => {
  console.log('Map Update Received: ', data)

  document.getElementById('map-tov').textContent = `Last Update: ${data.tov_s}:${data.tov_n}`
})

socket.on('map_error', (data) => {
  console.error('Map error:', data);
})