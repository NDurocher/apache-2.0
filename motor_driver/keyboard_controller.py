import rospy
from std_msgs.msg import String
import curses

print("Press 'q' to exit the loop.")

class KeyboardHandler:
    def __init__(self):
        # ROS setup
        rospy.init_node('keyboard_handler', anonymous=True)
        self.publisher = rospy.Publisher('keyboard_input', String, queue_size=10)

    def publish_key(self, key):
        # Publish the key press event to the ROS topic
        self.publisher.publish(key)

    def keyboard_input(self, stdscr):
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.clear()
        stdscr.addstr("Press 'q' to exit the loop.\n")

        key_map = {
            curses.KEY_UP: "up",
            curses.KEY_DOWN: "down",
            curses.KEY_LEFT: "left",
            curses.KEY_RIGHT: "right"
        }

        while not rospy.is_shutdown():
            try:
                key = stdscr.getch()
                if key != -1:  # Key was pressed
                    if key == ord('q'):
                        break
                    elif key in key_map:  # Handle arrow keys
                        self.publish_key(key_map[key])
                        # stdscr.addstr(f"You pressed: {key_map[key]}\n")
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
