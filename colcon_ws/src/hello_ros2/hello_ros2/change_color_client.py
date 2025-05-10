# service client 만들기
# 1. service 이름 타입 (turtulesim_node color 변경) 
# 실시간으로 다양한 색상으로 나타나게하기

# 2. launch 파일에 노드 추가  moveturtle.launch.py


# 3. move turtle.py -> 파마리터를 설정 각 속도 
# 움직을 외부에서 변화
# 4. 위 파라미터도  yaml에 넣어서 작동


"""
son@samson:~/WeGo_LIMO/colcon_ws$ ros2 interface show turtlesim/srv/SetPen 
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---

이것을 조정해서 변경해야한다.


server : turtlesim_node
client : 색상을 그리는 형식으로 보내는 것
"""


import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv  import SetPen

class change_color_client(Node):
    def __init__(self) :
        super().__init__("change_color_client")
        


        self.client = self.create_client(SetPen, "/turtle1/set_pen")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen 서비스 준비 중')


        self.count = 0
        self.val = 0.0

        # 1초마다 색 변경
        self.create_timer(1.0, self.pen_callback)
        
        

        self.color_toggle = False # 색상 토글

        # self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel" , 10)
        # self.create_timer(0.1, self.pub_turtle)

    # def pub_turtle(self):
    #     msg = Twist() # 클래스 객체로 해야함
    #     msg.angular.z= 1.0  ## python type 캐스팅이 자유롭다.
    #     # msg.linear.x = 2.0
    #     msg.linear.x = self.val  ## 하지만 dds로 넘길때 type check가 되어야한다.
    #     self.pub.publish(msg)
    #     self.val += 0.005


    def pen_callback(self):
        """펜 색·두께 설정 요청을 보낸다."""
        req = SetPen.Request()

        if not self.color_toggle:
            req.r = 0
            req.g = 0
            req.b = 255
            req.width = 3
            req.off = 0
        else:
            req.r = 255
            req.g = 0
            req.b = 0
            req.width = 3
            req.off = 0

        # 비동기 호출
        future = self.client.call_async(req)
        future.add_done_callback(self.pen_response)

        self.color_toggle = not self.color_toggle   # 다음 호출 때 색 반전


        # ── ★ 누락됐던 응답 처리 메서드 ★ ──
    def pen_response(self, future):
        res = future.result()
        


def main() :
    rp.init()
    node = change_color_client()

    try :
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

    
    

if __name__ == "__main__" :
    main()