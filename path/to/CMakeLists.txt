cmake_minimum_required(VERSION 3.8)
project(ball_detector)

# ament_auto のパッケージを検索
find_package(ament_auto REQUIRED)
ament_auto_find_build_dependencies()

# ライブラリの追加
ament_auto_add_library(ball_detector_lib
  src/ball_detector.cpp
  src/clustering.cpp
)

# 実行ファイルの追加
ament_auto_add_executable(ball_detector_node src/ball_detector.cpp)

# ライブラリと実行ファイルのリンク
target_link_libraries(ball_detector_node PRIVATE ball_detector_lib)

# アップパッケージ用の設定
ament_auto_package()
