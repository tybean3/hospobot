// Connect to ROS
const ros = new ROSLIB.Ros({
    url: `ws://${window.location.hostname}:9090`
});

const connectionStatus = document.getElementById('connection-status');
const mappingStatus = document.getElementById('mapping-status');
const notificationArea = document.getElementById('notification-area');

// ROS connection events
ros.on('connection', function() {
    connectionStatus.textContent = 'Connected';
    connectionStatus.className = 'status-badge connected';
    showNotification('Connected to Hospobot ROS 2 Bridge');
});

ros.on('error', function(error) {
    connectionStatus.textContent = 'Error';
    connectionStatus.className = 'status-badge disconnected';
    showNotification('Error connecting to websocket server', true);
});

ros.on('close', function() {
    connectionStatus.textContent = 'Disconnected';
    connectionStatus.className = 'status-badge disconnected';
});

// Services
const backupService = new ROSLIB.Service({
    ros: ros,
    name: '/rtabmap/backup',
    serviceType: 'std_srvs/srv/Empty'
});

const pauseService = new ROSLIB.Service({
    ros: ros,
    name: '/rtabmap/pause',
    serviceType: 'std_srvs/srv/Empty'
});

const resumeService = new ROSLIB.Service({
    ros: ros,
    name: '/rtabmap/resume',
    serviceType: 'std_srvs/srv/Empty'
});

const loadDatabaseService = new ROSLIB.Service({
    ros: ros,
    name: '/rtabmap/load_database',
    serviceType: 'rtabmap_msgs/srv/LoadDatabase'
});

// Helper functions
function showNotification(msg, isError = false) {
    notificationArea.textContent = msg;
    notificationArea.style.color = isError ? 'var(--danger)' : '#60a5fa';
    notificationArea.style.borderColor = isError ? 'rgba(239, 68, 68, 0.3)' : 'rgba(59, 130, 246, 0.3)';
    notificationArea.style.background = isError ? 'rgba(239, 68, 68, 0.2)' : 'rgba(59, 130, 246, 0.2)';
    notificationArea.classList.remove('hidden');
    
    setTimeout(() => {
        notificationArea.classList.add('hidden');
    }, 3000);
}

// Button Listeners
document.getElementById('btn-save').addEventListener('click', () => {
    showNotification('Saving map backup...');
    const request = new ROSLIB.ServiceRequest({});
    backupService.callService(request, (result) => {
        showNotification('Map successfully backed up!');
    }, (error) => {
        showNotification('Failed to backup map.', true);
        console.error(error);
    });
});

document.getElementById('btn-pause').addEventListener('click', () => {
    const request = new ROSLIB.ServiceRequest({});
    pauseService.callService(request, (result) => {
        mappingStatus.textContent = 'Mapping Paused';
        mappingStatus.className = 'status-text paused';
        showNotification('Mapping paused.');
    });
});

document.getElementById('btn-resume').addEventListener('click', () => {
    const request = new ROSLIB.ServiceRequest({});
    resumeService.callService(request, (result) => {
        mappingStatus.textContent = 'Mapping Active';
        mappingStatus.className = 'status-text active';
        showNotification('Mapping resumed.');
    });
});

document.getElementById('btn-revert').addEventListener('click', () => {
    if(!confirm("Are you sure you want to revert to the last saved map? All unsaved progress will be lost!")) {
        return;
    }
    
    showNotification('Reverting map database...');
    
    const request = new ROSLIB.ServiceRequest({
        database_path: '/home/hospobot/.ros/rtabmap.db.back',
        clear: true
    });
    
    loadDatabaseService.callService(request, (result) => {
        showNotification('Map reverted successfully!');
    }, (error) => {
        showNotification('Failed to revert map. Is there a backup?', true);
        console.error(error);
    });
});
