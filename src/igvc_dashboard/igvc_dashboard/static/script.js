

// Global Variables
// ------------------------------------------------

// init socketio
const socket = io()

const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

map = null
path = null
pos = null
goal = null


// Helpers
// -----------------------------------------------
function update_display() {
  const CELLSIZE = canvas.width / 10

  ctx.clearRect(0, 0, canvas.width, canvas.height)

  if (map) {

    if(goal) {
      map[goal.y * 10 + goal.x] = 'G'
    }
    if(pos) {
      map[pos.y * 10 + pos.x] = 'P'
    }
    if (path) {

    }


    for ( let y = 0; y < 10; y++) {
      for ( let x = 0; x < 10; x++) {
        const i = y * 10 + x
        const val = map[i]

        // Obstacle
        if (val == 1) {
          ctx.fillStyle = 'black'
        }
        // Goal
        else if ( val == 'G') {
          ctx.fillStyle = 'red'
        }
        // Position
        else if ( val == 'P') {
          ctx.fillStyle = 'green'
        }
        else {
          ctx.fillStyle = '#ddd'
        }

        ctx.fillRect(x * CELLSIZE, y * CELLSIZE, CELLSIZE, CELLSIZE)
      }
    }
  }

}

function update_display_tov(tov_s, tov_n) {
  document.getElementById('map-tov').textContent = `Last Update: ${tov_s}:${tov_n}`
}


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
  const { data: cells, tov_s, tov_n } = data
  update_display_tov(tov_s, tov_n)

  map = cells
  update_display()
})

socket.on('goal_update', (data) => {
  console.log('Goal Update Recieved: ', data)
  goal = data
  update_display()
})

socket.on('pos_update', (data) => {
  console.log('Pos Update Received: ', data)
  pos = data
  update_display()
})

socket.on('path_update', (data) => {
  console.log('Path Update Received: ', data)
  path = data
  update_display()
})

socket.on('error', (data) => {
  console.error('Error:', data);
})