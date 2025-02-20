import os
import subprocess
import sys

def is_running_in_docker():
    try:
        if os.path.isfile("/.dockerenv"):
            return True
    except:
        return False

def main():
    images_dir = "/home/dji/ws/tools-OmniNxtSimulator/apps/data_apps/output/cam_calib_sdg"
    bag_name = "calib_images.bag"

    # If not in Docker, run the Docker container
    if not is_running_in_docker():
        docker_cmd = [
            "docker", "run", "-it", "--rm",
            "-v", "{}:/images:rw".format(images_dir),
            "-v", "{}:/work/images2ros1bag.py:ro".format(__file__),
            "ros:noetic-perception-focal",
            "bash", "-c",
            "source /opt/ros/noetic/setup.bash && python3 /work/images2ros1bag.py"
        ]
        subprocess.run(docker_cmd)
        sys.exit(0)

    roscore_proc = subprocess.Popen(["roscore"])

    import rosbag
    from cv_bridge import CvBridge
    import cv2
    import rospy
    from sensor_msgs.msg import Image

    rospy.init_node("images2ros1bag")
    start_time = rospy.Time.now()
    fps = 5
    delta_t = 1.0 / fps
    timestamp = start_time
    bag = rosbag.Bag("/images/{}".format(bag_name), "w")
    bridge = CvBridge()
    for file_name in sorted(os.listdir("/images")):
        if file_name.lower().endswith(".png"):
            print("Processing: ", file_name)
            file_path = os.path.join("/images", file_name)
            cv_img = cv2.imread(file_path)
            msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            timestamp = timestamp + rospy.Duration.from_sec(delta_t)
            msg.header.stamp = timestamp
            bag.write("/calib_images", msg)
    bag.close()
    print("Created {} with topic /calib_images.".format(bag_name))
    roscore_proc.terminate()

if __name__ == "__main__":
    main()