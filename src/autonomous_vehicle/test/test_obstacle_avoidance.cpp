// Kiểm thử logic tốc độ và state machine của ObstacleAvoidance.
//
// Trọng tâm là phần tốc độ, vì đó là thay đổi hành vi lái: trước đây NORMAL
// gán cứng 35 và bộ lập của detector bị bỏ không.
//
//   colcon test --packages-select autonomous_vehicle
//
#include <gtest/gtest.h>

#include <limits>

#include "lidar_module.hpp"
#include "obstacle_avoidance.hpp"

namespace {

// LiDAR trống: không có vật cản nào, nhưng has_data = true để không bị coi
// là mất cảm biến.
LidarStatus clearLidar() {
    LidarStatus st;
    st.has_data = true;
    st.alert = "CLEAR";
    st.detail = "All clear";
    return st;
}

}  // namespace

// -----------------------------------------------------------------------------
// TỐC ĐỘ NORMAL LẤY TỪ DETECTOR
// -----------------------------------------------------------------------------

// Đường thẳng: detector yêu cầu 85 (8.5 km/h) -> NORMAL phải phát 85.
TEST(ObstacleSpeed, NormalUsesLanePlannerSpeed) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/85,
        false, false);

    EXPECT_EQ(c.state, BypassState::NORMAL);
    EXPECT_EQ(c.speed_control, 85);
}

// Vào cua: detector hạ xuống 60 -> NORMAL phải theo, không phải giữ 85.
TEST(ObstacleSpeed, NormalFollowsCurveSpeedDown) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/60,
        false, false);

    EXPECT_EQ(c.state, BypassState::NORMAL);
    EXPECT_EQ(c.speed_control, 60);
}

// Mất lane (detector trả 0) -> chạy chậm, KHÔNG dừng.
TEST(ObstacleSpeed, NoLaneKeepsMovingSlowly) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, false, "NONE", 0.0f, /*lane_speed_x10=*/0,
        false, false);

    EXPECT_EQ(c.state, BypassState::NORMAL);
    EXPECT_EQ(c.speed_control, SPEED_NO_LANE_X10);
    EXPECT_GT(c.speed_control, 0) << "mất lane thì vẫn phải chạy chậm, không dừng";
    EXPECT_FALSE(c.emergency_stop);
}

// Giá trị vượt trần bị kẹp lại.
TEST(ObstacleSpeed, ClampsToNormalMax) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/200,
        false, false);

    EXPECT_EQ(c.speed_control, SPEED_NORMAL_MAX_X10);
}

// Tham số override: đặt 35 là quay lại hành vi cũ, bỏ để detector quyết.
TEST(ObstacleSpeed, OverrideWinsOverLanePlanner) {
    ObstacleAvoidance oa;
    ObstacleAvoidance::set_speed_normal_override(35);
    {
        const BypassCommand c = oa.update(
            clearLidar(), 0, 0.0f, true, "NONE", 0.0f, 85, false, false);
        EXPECT_EQ(c.speed_control, 35);
    }
    ObstacleAvoidance::set_speed_normal_override(0);

    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE", 0.0f, 85, false, false);
    EXPECT_EQ(c.speed_control, 85);
}

// -----------------------------------------------------------------------------
// STATE LÁCH: tốc độ do logic an toàn quyết, KHÔNG theo detector
// -----------------------------------------------------------------------------

// Vật cản phía trước -> SLOW_DOWN ở 30, không phải 85 của detector.
TEST(ObstacleSpeed, SlowDownIgnoresLanePlannerSpeed) {
    ObstacleAvoidance oa;
    LidarStatus st = clearLidar();
    st.ob_front_cm = 100.0f;   // trong ngưỡng 170 nhưng ngoài 140 -> SLOW_DOWN

    const BypassCommand c = oa.update(
        st, 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/85, false, false);

    EXPECT_EQ(c.state, BypassState::SLOW_DOWN);
    EXPECT_EQ(c.speed_control, SPEED_BYPASS_X10);
    EXPECT_LT(c.speed_control, 85);
}

// Vật cản rất gần -> EMERGENCY_STOP, tốc độ 0.
TEST(ObstacleSpeed, EmergencyStopsOnCloseObstacle) {
    ObstacleAvoidance oa;
    LidarStatus st = clearLidar();
    st.ob_front_cm = 30.0f;

    const BypassCommand c = oa.update(
        st, 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/85, false, false);

    EXPECT_EQ(c.state, BypassState::EMERGENCY_STOP);
    EXPECT_EQ(c.speed_control, SPEED_HOLD_X10);
    EXPECT_TRUE(c.emergency_stop);
}

// -----------------------------------------------------------------------------
// AN TOÀN CẢM BIẾN
// -----------------------------------------------------------------------------

// Mất LiDAR -> dừng, bất kể detector yêu cầu gì.
TEST(ObstacleSafety, LidarStaleStopsRegardlessOfLaneSpeed) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE", 0.0f, /*lane_speed_x10=*/85,
        /*camera_stale=*/false, /*lidar_stale=*/true);

    EXPECT_EQ(c.state, BypassState::EMERGENCY_STOP);
    EXPECT_EQ(c.speed_control, SPEED_HOLD_X10);
    EXPECT_TRUE(c.lidar_stale);
}

// Mất camera -> tốc độ 0, lái thẳng, LiDAR vẫn lo phần vật cản.
TEST(ObstacleSafety, CameraStaleStopsButKeepsLidar) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 40, 0.0f, true, "NONE", 0.0f, 85,
        /*camera_stale=*/true, /*lidar_stale=*/false);

    EXPECT_EQ(c.speed_control, SPEED_HOLD_X10);
    EXPECT_TRUE(c.camera_stale);
    EXPECT_FALSE(c.lidar_stale);
}

// Đèn đỏ -> dừng.
TEST(ObstacleSafety, RedLightStops) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, /*traffic_light_decision=*/"RED", 0.0f, 85,
        false, false);

    EXPECT_EQ(c.state, BypassState::EMERGENCY_STOP);
    EXPECT_EQ(c.speed_control, SPEED_HOLD_X10);
    EXPECT_TRUE(c.emergency_stop);
}

// Tốc độ đọc từ ESP32 sai (NaN) không được làm hỏng logic.
TEST(ObstacleSafety, NonFiniteSpeedIsHandled) {
    ObstacleAvoidance oa;
    const BypassCommand c = oa.update(
        clearLidar(), 0, 0.0f, true, "NONE",
        std::numeric_limits<float>::quiet_NaN(), 85, false, false);

    EXPECT_EQ(c.state, BypassState::NORMAL);
    EXPECT_EQ(c.speed_control, 85);
}