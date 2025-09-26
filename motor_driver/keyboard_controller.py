import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import curses

print("Press 'q' to exit the loop.")

class KeyboardHandler:
    def __init__(self):
        # ROS setup
        rospy.init_node('keyboard_handler', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def publish_key(self, key):
        # Convert the key press event to velocity command and publish to ROS topic
	vel_command = Twist()
	if key == "up":
	  vel_command.linear.x = 1.0
	  vel_command.angular.z = 0.0
	elif key == "down":
	  vel_command.linear.x = -1.0
	  vel_command.angular.z = 0.0
	elif key == "left":
          vel_command.linear.x = 0.0
	  vel_command.angular.z = 1.0
	elif key == "right":
          vel_command.linear.x = 0.0
	  vel_command.angular.z = -1.0
	elif key == "end":
	  vel_command.linear.x = 0.0
          vel_command.angular.z = 0.0
        
	self.publisher.publish(vel_command)

    def keyboard_input(self, stdscr):
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.clear()
        stdscr.addstr("Press 'q' to exit the loop.\n")

        key_map = {
            curses.KEY_UP: "up",
            curses.KEY_DOWN: "down",
            curses.KEY_LEFT: "left",
            curses.KEY_RIGHT: "right",
	    curses.KEY_END: "end"
        }

        while not rospy.is_shutdown():
            try:
                key = stdscr.getch()
                if key != -1:  # Key was pressed
                    if key == ord('q'):
                        break
                    elif key in key_map:  # Handle arrow keys
                        self.publish_key(key_map[key])
                        print("You pressed: {}").format(key)
                    else:
                        self.publish_key(chr(key))  # Publish other keys
                        # stdscr.addstr(f"You pressed: {chr(key)}\n")
                    stdscr.refresh()
            except Exception as e:
                rospy.logerr("Error")

if __name__ == "__main__":
    kb_handler = KeyboardHandler()
    try:
        curses.wrapper(kb_handler.keyboard_input)
    except rospy.ROSInterruptException:
        pass
