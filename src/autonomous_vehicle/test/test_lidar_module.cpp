// Kiểm thử địa lý cung đo LiDAR: đặt vật cản ở từng góc trong khung xe và
// xác nhận rằng nó rơi đúng vào cung mong đợi.
//
//   colcon test --packages-select autonomous_vehicle
//
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "lidar_module.hpp"

namespace {

constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;

// Dựng một LaserScan giả: góc 0 -> 360 độ, bước `step_deg`, mỗi góc đặt một
// vật cản ở khoảng cách `dist_cm` (theo khung xe sau khi hiệu chỉnh).
sensor_msgs::msg::LaserScan makeScan(
    LidarModule& mod, float step_deg, float obstacle_deg_vehicle,
    float obstacle_cm)
{
    sensor_msgs::msg::LaserScan scan;
    const int n = static_cast<int>(std::ceil(360.0f / step_deg));
    scan.angle_min = 0.0f;
    scan.angle_max = static_cast<float>(n) * step_deg * DEG2RAD;
    scan.angle_increment = step_deg * DEG2RAD;
    scan.range_min = 0.0f;
    scan.range_max = 25.0f;
    scan.ranges.assign(static_cast<size_t>(n), std::numeric_limits<float>::infinity());

    // offset = -90 nghĩa là góc thô = góc xe + 90
    const float raw_deg = std::fmod(obstacle_deg_vehicle - mod.get_mount_offset_deg() + 360.0f, 360.0f);
    const int idx = static_cast<int>(std::lround(raw_deg / step_deg)) % n;
    scan.ranges[static_cast<size_t>(idx)] = obstacle_cm * 0.01f;

    return scan;
}

LidarStatus runAt(LidarModule& mod, float deg, float cm) {
    LidarStatus st;
    const auto scan = makeScan(mod, 0.5f, deg, cm);
    mod.update(scan, st);
    return st;
}

} // namespace

// ---------------------------------------------------------------------------
// Vật cản thẳng trước xe -> CHỈ chạm ob_front, không chạm hai cung bên.
// Đây là hồi quy trực tiếp cho lỗi A1 của bản cũ.
// ---------------------------------------------------------------------------
TEST(LidarSector, FrontObstacleOnlyHitsFrontSector) {
    LidarModule mod;
    const LidarStatus st = runAt(mod, 0.0f, 80.0f);

    ASSERT_TRUE(st.has_data);
    ASSERT_TRUE(st.ob_front_cm.has_value());
    EXPECT_NEAR(*st.ob_front_cm, 80.0f, 1.0f);
    EXPECT_FALSE(st.ob_left_cm.has_value());
    EXPECT_FALSE(st.ob_right_cm.has_value());
    EXPECT_NEAR(*st.front_min_cm, 80.0f, 1.0f);
}

// Vật cản ở bên trái -> ob_left, không chạm ob_front.
TEST(LidarSector, LeftObstacleHitsLeftSectorOnly) {
    LidarModule mod;
    const LidarStatus st = runAt(mod, 90.0f, 100.0f);

    ASSERT_TRUE(st.ob_left_cm.has_value());
    EXPECT_NEAR(*st.ob_left_cm, 100.0f, 1.0f);
    EXPECT_FALSE(st.ob_front_cm.has_value());
    EXPECT_FALSE(st.ob_right_cm.has_value());
    EXPECT_NEAR(*st.left_min_cm, 100.0f, 1.0f);
}

// Vật cản ở bên phải -> ob_right, không chạm ob_front.
TEST(LidarSector, RightObstacleHitsRightSectorOnly) {
    LidarModule mod;
    const LidarStatus st = runAt(mod, 270.0f, 100.0f);

    ASSERT_TRUE(st.ob_right_cm.has_value());
    EXPECT_NEAR(*st.ob_right_cm, 100.0f, 1.0f);
    EXPECT_FALSE(st.ob_front_cm.has_value());
    EXPECT_FALSE(st.ob_left_cm.has_value());
    EXPECT_NEAR(*st.right_min_cm, 100.0f, 1.0f);
}

// Vật cản phía sau -> không chạm cung trước.
TEST(LidarSector, RearObstacleDoesNotHitFront) {
    LidarModule mod;
    const LidarStatus st = runAt(mod, 180.0f, 90.0f);

    EXPECT_FALSE(st.ob_front_cm.has_value());
    ASSERT_TRUE(st.ob_rear_cm.has_value());
    EXPECT_NEAR(*st.ob_rear_cm, 90.0f, 1.0f);
    EXPECT_NEAR(*st.rear_bypass_min_cm, 90.0f, 1.0f);
}

// Cung sau-trái / sau-phải là hai vùng riêng biệt (bản cũ chồng lấn 160°).
TEST(LidarSector, RearLeftAndRearRightAreDisjoint) {
    LidarModule mod;

    const LidarStatus l = runAt(mod, 157.0f, 70.0f);
    ASSERT_TRUE(l.ob_left_rear_cm.has_value());
    EXPECT_FALSE(l.ob_right_rear_cm.has_value());

    const LidarStatus r = runAt(mod, 203.0f, 70.0f);
    ASSERT_TRUE(r.ob_right_rear_cm.has_value());
    EXPECT_FALSE(r.ob_left_rear_cm.has_value());
}

// Quét toàn vòng: mỗi góc phải thuộc đúng một nhóm cung.
TEST(LidarSector, EveryAngleMapsToConsistentSector) {
    LidarModule mod;
    for (float deg = 0.0f; deg < 360.0f; deg += 5.0f) {
        const LidarStatus st = runAt(mod, deg, 100.0f);
        ASSERT_TRUE(st.has_data) << "no data at " << deg;

        int ob_sectors = 0;
        if (st.ob_front_cm)   ++ob_sectors;
        if (st.ob_left_cm)    ++ob_sectors;
        if (st.ob_right_cm)   ++ob_sectors;
        if (st.ob_rear_cm)    ++ob_sectors;
        EXPECT_EQ(ob_sectors, 1) << "angle " << deg << " matched " << ob_sectors << " sectors";

        int hud_sectors = 0;
        if (st.front_min_cm)       ++hud_sectors;
        if (st.left_min_cm)        ++hud_sectors;
        if (st.rear_bypass_min_cm) ++hud_sectors;
        if (st.right_min_cm)       ++hud_sectors;
        EXPECT_EQ(hud_sectors, 1) << "angle " << deg << " matched " << hud_sectors << " HUD sectors";
    }
}

// Góc hiệu chỉnh khác -> cung dịch chuyển tương ứng.
TEST(LidarSector, MountOffsetShiftsSectors) {
    LidarModule mod;
    ASSERT_EQ(mod.get_mount_offset_deg(), LidarModule::DEFAULT_MOUNT_OFFSET_DEG);

    // offset = 0 => góc thô = góc xe.
    mod.set_mount_offset_deg(0.0f);
    const LidarStatus st = runAt(mod, 0.0f, 50.0f);
    ASSERT_TRUE(st.ob_front_cm.has_value());
    EXPECT_NEAR(*st.ob_front_cm, 50.0f, 1.0f);
}

// Vật cản gần báo DANGER; xa báo CLEAR.
TEST(LidarSector, AlertLevels) {
    LidarModule mod;
    EXPECT_EQ(runAt(mod, 0.0f, 30.0f).alert, "DANGER");
    EXPECT_EQ(runAt(mod, 0.0f, 50.0f).alert, "WARNING");
    EXPECT_EQ(runAt(mod, 0.0f, 150.0f).alert, "CLEAR");
}

// Scan rỗng => has_data = false, coi như mất cảm biến.
TEST(LidarSector, EmptyScanReportsNoData) {
    LidarModule mod;
    sensor_msgs::msg::LaserScan scan;
    scan.ranges.clear();
    LidarStatus st;
    mod.update(scan, st);
    EXPECT_FALSE(st.has_data);
}

// NaN / Inf bị bỏ qua, nhưng còn điểm khác thì vẫn has_data.
TEST(LidarSector, IgnoresNonFiniteRanges) {
    LidarModule mod;
    sensor_msgs::msg::LaserScan scan;
    const int n = 720;
    scan.angle_min = 0.0f;
    scan.angle_increment = (2.0f * 3.14159265f) / n;
    scan.range_min = 0.0f;
    scan.range_max = 25.0f;
    scan.ranges.assign(n, std::numeric_limits<float>::infinity());
    scan.ranges[100] = std::numeric_limits<float>::quiet_NaN();
    scan.ranges[200] = 0.5f;   // 50cm -> trong cả MAP_RADIUS (80) lẫn DETECT_RADIUS

    LidarStatus st;
    mod.update(scan, st);
    EXPECT_TRUE(st.has_data);
    EXPECT_EQ(st.points_px.size(), 1u);
}
