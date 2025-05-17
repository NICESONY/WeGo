cmake_minimum_required(VERSION 3.8)
project(user_interface)  # 패키지 이름

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# ROS 2 빌드·메시지 생성 도구
# -------------------------
find_package(ament_cmake           REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces   REQUIRED)   # ← 추가

# -------------------------
# msg/.srv 파일로 코드 생성
# -------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyTopic.msg"
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
  DEPENDENCIES std_msgs  builtin_interfaces                     
)

# -------------------------
# 테스트용 린터 (선택)
# -------------------------
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND    TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -------------------------
# 런타임 의존성 내보내기
# -------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
