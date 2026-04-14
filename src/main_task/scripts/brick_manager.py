from collections import defaultdict
import rospy

class BrickManager:
    def __init__(self):
        self.bricks = [
            ("I1", "yellow", 0.015, 0.0325, 0.0, 90),
            ("I2", "yellow", 0.015, 0.1075, 0.0, 90),
            ("I3", "yellow", 0.015, 0.1825, 0.0, 90),
            
            ("A1", "blue", 0.07, 0.0325, 0.0, 70),
            ("A2", "red", 0.0957, 0.1075, 0.0, 70),
            ("A4", "blue", 0.2386, 0.0325, 0.0, -70),
            ("A5", "red", 0.2129, 0.1075, 0.0, -70),
            ("A3", "red", 0.1214, 0.1825, 0.0, 70),
            ("A6", "red", 0.1872, 0.1825, 0.0, -70),
            ("A7", "blue", 0.155, 0.09, 0.0, 0),
            
            ("S1", "green", 0.33, 0.015, 0.0, 0),
            ("S2", "green", 0.38, 0.06, 0.0, 90),
            ("S3", "green", 0.33, 0.105, 0.0, 0),
            ("S4", "green", 0.27, 0.15, 0.0, 90),
            ("S5", "green", 0.33, 0.195, 0.0, 0),
        ]


        self.color_groups = defaultdict(list)
        for brick in self.bricks:
            name, color, x, y, z, c = brick
            self.color_groups[color].append((name, x, y, z, c))
        
        self.iterators = {
            color: iter(group)
            for color, group in self.color_groups.items()
        }

    def get_next_position(self, color):
        default_positon = (0.6,0.5,0.0)
        default_orientation = (180,0,-45)
        try:
            name, x, y, z, c = next(self.iterators[color])
            position    = (default_positon[0] - x, default_positon[1] - y, default_positon[2] - z)
            orientation = (default_orientation[0], default_orientation[1] , default_orientation[2] + c)
            rospy.loginfo(f"Position for Brick with colour {color}: {position,orientation}")
            return position, orientation
        except StopIteration:
            position    = ()
            orientation = ()
            rospy.loginfo(f"No more positon for Brick with colour: {color}, default Wert: {position,orientation}")
            return position, orientation
        except KeyError:
            position    = ()
            orientation = ()
            rospy.loginfo(f"Colour has no position: {color}, default Wert: {position,orientation}")
            return position, orientation



if __name__ == "__main__":
    rospy.init_node("BrickManager_Node", anonymous=True)
    # Beispielnutzung:
    manager = BrickManager()

    manager.get_next_position("yellow")  # (1.5, 3.25, 0, 90)
    manager.get_next_position("yellow")  # (1.5, 10.75, 0, 90)
    manager.get_next_position("yellow")  # (1.5, 18.25, 0, 90)
    manager.get_next_position("yellow")  # None
    manager.get_next_position("green")   # (33, 1.5, 0, 0)
    manager.get_next_position("blue")    # None
