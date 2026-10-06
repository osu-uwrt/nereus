// The prior map (riptide_mapping config.yaml): loading, edits, the comment-preserving save, the map origin and
// the AprilTag spots.
#include "prior_map.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

using namespace nereus::ros_viewer::host::prior_map;

namespace {
// The checked-in riptide_mapping config.yaml fixture, as text.
std::string fixture() {
    std::ifstream in(NEREUS_TEST_FIXTURE);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

std::vector<std::string> lines(const std::string &text) {
    std::vector<std::string> out;
    std::stringstream stream(text);
    for (std::string line; std::getline(stream, line);)
        out.push_back(line);
    return out;
}

// Lines of `b` that differ from `a` at the same index (the files must have the same length).
std::vector<std::string> changed(const std::string &a, const std::string &b) {
    const auto la = lines(a), lb = lines(b);
    EXPECT_EQ(la.size(), lb.size());
    std::vector<std::string> out;
    for (std::size_t i = 0; i < std::min(la.size(), lb.size()); ++i)
        if (la[i] != lb[i])
            out.push_back(lb[i]);
    return out;
}
} // namespace

// Only the Talos namespace is loaded, with "_frame" dropped from parent names and the file's editor flags read.
TEST(PriorMap, LoadsTheTalosSection) {
    const auto doc = load(fixture());
    EXPECT_EQ(doc.ns, "/talos/riptide_mapping2");
    EXPECT_EQ(doc.namespaces, std::vector<std::string>{"/talos/riptide_mapping2"}); // liltank is not offered

    ASSERT_TRUE(find(doc.objects, "gate"));
    EXPECT_EQ(find(doc.objects, "gate")->parent, "map");
    EXPECT_FALSE(find(doc.objects, "gate")->locked);
    const auto *rescue = find(doc.objects, "gate_rescue");
    ASSERT_TRUE(rescue);
    EXPECT_EQ(rescue->parent, "gate"); // "gate_frame" in the file
    EXPECT_TRUE(rescue->locked);       // children ride along with their assembly
    EXPECT_FALSE(find(doc.objects, "prequal_gate"));

    // At least one object carries a class and one points its yaw at its parent.
    bool classes = false, pointing = false;
    for (const auto &o : doc.objects) {
        classes = classes || o.cls == "fire";
        pointing = pointing || o.pointYawAtParent;
    }
    EXPECT_TRUE(classes && pointing);
}

TEST(PriorMap, UnchangedSaveIsByteIdentical) {
    const auto doc = load(fixture());
    EXPECT_EQ(save(doc), doc.text);
}

// Saving rewrites only the edited value lines; everything else in the file is kept verbatim.
TEST(PriorMap, EditsTouchOnlyTheirValues) {
    auto doc = load(fixture());
    find(doc.objects, "gate")->pose.x = 4.5;
    find(doc.objects, "torpedo")->pose.yaw = -150; // a whole number is still written as a float
    const auto out = save(doc);
    const auto diff = changed(doc.text, out);
    ASSERT_EQ(diff.size(), 2u);
    EXPECT_EQ(diff[0], "          x: 4.5");
    EXPECT_EQ(diff[1], "          yaw: -150.0");
    EXPECT_NEAR(find(load(out).objects, "gate")->pose.x, 4.5, 1e-9);
}

// Changing a class keeps the trailing comment on its line.
TEST(PriorMap, ClassesKeepTheirComments) {
    auto doc = load(fixture());
    Object *fire = nullptr;
    for (auto &o : doc.objects)
        if (o.cls == "fire" && !fire)
            fire = &o;
    ASSERT_TRUE(fire);
    fire->cls = "blood";
    const auto diff = changed(doc.text, save(doc));
    ASSERT_EQ(diff.size(), 1u);
    EXPECT_EQ(diff[0], "        class: blood   # SET PER RUN");
}

// Setting a flag or class the object lacked inserts a line; clearing them restores the original file exactly.
TEST(PriorMap, FlagsAndClassesAreAddedAndRemoved) {
    auto doc = load(fixture());
    auto *gate = find(doc.objects, "gate");
    gate->lockOrientation = true;
    gate->cls = "big";
    const auto added = save(doc);
    const auto back = load(added);
    EXPECT_TRUE(find(back.objects, "gate")->lockOrientation);
    EXPECT_EQ(find(back.objects, "gate")->cls, "big");
    EXPECT_EQ(lines(added).size(), lines(doc.text).size() + 2);

    // and off again: back to the original file
    auto again = load(added);
    find(again.objects, "gate")->lockOrientation = false;
    find(again.objects, "gate")->cls.clear();
    EXPECT_EQ(save(again), doc.text);
}

// New objects are written under their parent's "_frame" name, and deleting one keeps its children in place.
// Sections the editor doesn't show (deprecated entries, other robots) survive the save.
TEST(PriorMap, AddedAndDeletedObjects) {
    auto doc = load(fixture());
    const auto name = add(doc.objects, "gate", {5, 1, -1, 90});
    EXPECT_EQ(name, "prop");
    remove(doc.objects, "torpedo"); // its holes move to the map where they are
    const auto torpedoChildren = descendants(load(fixture()).objects, "torpedo");
    ASSERT_FALSE(torpedoChildren.empty());
    for (const auto &child : torpedoChildren)
        EXPECT_EQ(find(doc.objects, child)->parent, "map");

    const auto out = save(doc);
    const auto back = load(out);
    EXPECT_FALSE(find(back.objects, "torpedo"));
    ASSERT_TRUE(find(back.objects, "prop"));
    EXPECT_EQ(find(back.objects, "prop")->parent, "gate");
    EXPECT_NE(out.find("      prop:\n        parent: gate_frame\n"), std::string::npos);
    EXPECT_NE(out.find("prequal_gate:"), std::string::npos); // deprecated entries stay
    EXPECT_NE(out.find("/liltank/riptide_mapping2:"), std::string::npos);

    const auto mapped = mapPoses(back.objects);
    EXPECT_NEAR(mapped.at("prop").x, 5, 1e-5);
    EXPECT_NEAR(mapped.at("prop").yaw, 90, 1e-5);
}

// Reparent, swap, rename and duplicate: map poses are preserved and invalid edits are refused with a reason.
TEST(PriorMap, TreeEditsKeepPoolPositions) {
    auto doc = load(fixture());
    const auto before = mapPoses(doc.objects);
    ASSERT_TRUE(reparent(doc.objects, "gate_rescue", "map"));
    EXPECT_NEAR(mapPoses(doc.objects).at("gate_rescue").x, before.at("gate_rescue").x, 1e-9);
    std::string error;
    EXPECT_FALSE(reparent(doc.objects, "gate", "gate_repair", &error)); // its own descendant
    EXPECT_FALSE(error.empty());

    ASSERT_TRUE(swapPoses(doc.objects, "gate", "torpedo"));
    const auto after = mapPoses(doc.objects);
    EXPECT_NEAR(after.at("gate").x, before.at("torpedo").x, 1e-9);
    EXPECT_NEAR(after.at("torpedo").yaw, before.at("gate").yaw, 1e-9);

    ASSERT_TRUE(rename(doc.objects, "gate", "gate2"));
    EXPECT_EQ(find(doc.objects, "gate_repair")->parent, "gate2");
    EXPECT_FALSE(rename(doc.objects, "gate2", "torpedo", &error)); // name taken
    const auto copy = duplicate(doc.objects, "gate2");
    EXPECT_EQ(copy, "gate2_copy");
}

// Poses are x, y, z and yaw in degrees; yaw wraps into (-180, 180].
TEST(PriorMap, ComposeAndDecomposeAreInverse) {
    const Pose parent{3, -2, -1, 135}, child{1.5, .5, -.25, -170};
    const auto inMap = compose(parent, child);
    const auto back = decompose(parent, inMap);
    EXPECT_NEAR(back.x, child.x, 1e-12);
    EXPECT_NEAR(back.y, child.y, 1e-12);
    EXPECT_NEAR(back.z, child.z, 1e-12);
    EXPECT_NEAR(back.yaw, child.yaw, 1e-9);
    EXPECT_DOUBLE_EQ(wrapDegrees(-180), 180);
    EXPECT_DOUBLE_EQ(wrapDegrees(540), 180);
}

// keepInPool re-expresses the objects under a moved origin so they stay put in the pool; mapToPool and
// poolToMap are inverse.
TEST(PriorMap, OriginMovesWithOrWithoutTheObjects) {
    auto doc = load(fixture());
    Origin from;
    from.x = 1;
    from.y = 2;
    from.basePhi = 90;
    Origin to = from;
    to.x = 10;
    to.yawOffset = -30;

    const auto before = mapToPool(find(doc.objects, "gate")->pose, from);
    keepInPool(doc.objects, from, to);
    const auto after = mapToPool(find(doc.objects, "gate")->pose, to);
    EXPECT_NEAR(after.x, before.x, 1e-9);
    EXPECT_NEAR(after.y, before.y, 1e-9);
    EXPECT_NEAR(after.yaw, before.yaw, 1e-9);

    const auto round = poolToMap(mapToPool({1, 2, -1, 30}, to), to);
    EXPECT_NEAR(round.x, 1, 1e-12);
    EXPECT_NEAR(round.yaw, 30, 1e-9);
}

TEST(PriorMap, TagSpotsAtLineEndsAndCorners) {
    // a line along the length at y = 5, one across at x = 10, and a T bar (skipped)
    const std::vector<Line> lines{{2, 5, 48, 5}, {10, 2, 10, 20}, {9.5, 2, 10.5, 2}};
    const auto spots = tagSpots(50, 22, lines);
    EXPECT_EQ(spots.size(), 4u + 8u); // two ends per long line, four corners once per wall

    const auto west = nearestSpot(spots, .5, 5.3);
    ASSERT_TRUE(west);
    EXPECT_EQ(west->wall, 'W');
    EXPECT_DOUBLE_EQ(west->phi, 0); // +X into the pool
    const auto north = nearestSpot(spots, 10.4, 21.5);
    ASSERT_TRUE(north);
    EXPECT_EQ(north->wall, 'N');
    EXPECT_DOUBLE_EQ(north->phi, 270);

    // at a corner, the wall the point hugs
    EXPECT_EQ(nearestSpot(spots, .2, 1.0)->wall, 'W');
    EXPECT_EQ(nearestSpot(spots, 1.0, .2)->wall, 'S');
    EXPECT_FALSE(nearestSpot(spots, 25, 11));
}
