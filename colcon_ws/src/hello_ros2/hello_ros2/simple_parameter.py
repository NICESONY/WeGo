import rclpy as rp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
"""
ros2 run <패키지> <노드 실행파일> --ros-args -p use_sim_time:=True

ros2 run hello_ros2 /simple_parameter --ros-args -p use_sim_time:=True


## 중요
ros2 param set 노드이름 파라미터이름 변경값
ros2 param get 노드이름 파라미터이름


ros2 param set turtlesim  background_r 100

ros2 param set simple_parameter para1 1000

"""

class simple_parameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.create_timer(1, self.update)
        self.para1 = 0
        self.declare_parameter("para1", 0) 
        self.para1 = self.get_parameter("para1").get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.parameter_callback)


    def update(self) :
        self.get_logger().info(f" parameter{self.para1}")
        self.para1 += 1
        self.set_parameters([Parameter('para1', Parameter.Type.INTEGER, self.para1)])


    def parameter_callback(self, parameters : list[Parameter]):
        for parameter in parameters :
            if parameter.name == "para1":
                self.para1 : int = int(parameter.value)
        # self.para1 = self.get_parameter("para1").get_parameter_value().integer_value
        return SetParametersResult(successful = True)



        
def main():
    rp.init()
    node = simple_parameter()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()

if __name__ == "__main__":
    main()