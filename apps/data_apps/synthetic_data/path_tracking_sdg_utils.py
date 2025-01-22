# import data_apps.synthetic_data.scene_based_sdg_utils as scene_based_sdg_utils
import h5py
import numpy as np

class Trajectory:
    def __init__(self, group, mass=0.85, radius=0.15):
        self.group = group
        self.acc = group['acc'][:]
        self.jerk = group['jerk'][:]
        self.omg = group['omg'][:]
        self.pos = group['pos'][:]
        self.quaternion = group['quaternion'][:]
        self.start_goal = group['start_goal'][:]
        self.thr = group['thr'][:]
        self.timestamp = group['timestamp'][:]
        self.vel = group['vel'][:]
        self.mass = mass
        self.radius = radius
    
    def __len__(self):
        return len(self.timestamp)

    def __getitem__(self, key):
        return Trajectory(self.group[key])
    
    def getAccbyIndex(self, idx):
        return self.acc[idx]
    
    def getJerkbyIndex(self, idx):
        return self.jerk[idx]
    
    def getOmgbyIndex(self, idx):
        return self.omg[idx]
    
    def getPosbyIndex(self, idx):
        return self.pos[idx]
    
    def getQuaternionbyIndex(self, idx):
        return self.quaternion[idx]
    
    def getThrbyIndex(self, idx):
        return self.thr[idx]
    
    def getVelbyIndex(self, idx):
        return self.vel[idx]
    
    def getAccbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.acc[idx]

    def getJerkbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.jerk[idx]
    
    def getOmgbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.omg[idx]
    
    def getPosbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.pos[idx]

    def getQuaternionbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.quaternion[idx]
    
    def getThrbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.thr[idx]
    
    def getVelbyTimestamp(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        return self.vel[idx]
    
    def getStartGoal(self):
        return self.start_goal
    
    def getTimestamp(self):
        return self.timestamp
    
    def getQuadrotorForcesAndTorques(self, timestamp):
        idx = np.argmin(np.abs(self.timestamp - timestamp))
        F = self.mass * self.acc[idx] + np.array([0, 0, self.mass * 9.81])
        I = 0.5 * self.mass * np.array([self.radius**2, self.radius**2, self.radius**2])
        if idx == 0:
            omega_acc = (self.omg[idx] - np.array([0, 0, 0])) / 0.1
        else:
            omega_acc = (self.omg[idx] - self.omg[idx-1]) / (self.timestamp[idx] - self.timestamp[idx-1])
        tau = I * omega_acc
        return F, tau
        
class TrajectoryDatabase:
    def __init__(self, file_path, mass=0.85, radius=0.15):
        self.file_path = file_path
        self.file = h5py.File(file_path, 'r')
        self.mass = mass
        self.radius = radius
        self.trajectories = [Trajectory(self.file[str(i)], self.mass, self.radius) for i in range(len(self.file))]
        
    
    def __len__(self):
        return len(self.trajectories)
    
    def getTrajectory(self, idx):
        return self.trajectories[idx]

# Only for testing
if __name__ == '__main__':
    file_path = '/workspace/isaaclab/user_apps/assets/trajectory.hdf5'
    db = TrajectoryDatabase(file_path)
    print('Number of trajectories:', len(db))
    print('Number of samples in the first trajectory:', len(db.getTrajectory(0)))

    # Get the first trajectory information at timestamp t
    timestamp = 4.58
    trajectory = db.getTrajectory(0)
    print(f'Information at timestamp {timestamp}:')
    print('Position:', trajectory.getPosbyTimestamp(timestamp))
    print('Velocity:', trajectory.getVelbyTimestamp(timestamp))
    print('Acceleration:', trajectory.getAccbyTimestamp(timestamp))
    print('Jerk:', trajectory.getJerkbyTimestamp(timestamp))
    print('Angular velocity:', trajectory.getOmgbyTimestamp(timestamp))
    print('Quaternion:', trajectory.getQuaternionbyTimestamp(timestamp))
    print('Start and goal:', trajectory.getStartGoal())