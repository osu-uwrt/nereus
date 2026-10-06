// The TF panel's frame tree: per-frame visibility checkboxes, branch selection, and how both survive TF updates.
#include "tf_tree.hpp"
#include <gtest/gtest.h>
using namespace nereus::ros_viewer::host;

// Branch (de)selection, per-frame checkboxes, frames that reappear, new frames after "select none", and cycles.
TEST(HostTfTree, SelectionSurvivesReparenting) {
    TfTree tree;
    std::map<std::string, std::string> parents{{"map", ""},
                                               {"gate_frame", "map"},
                                               {"target_frame", "gate_frame"},
                                               {"talos/base_link", "map"},
                                               {"camera", "talos/base_link"},
                                               {"disconnected", ""},
                                               {"sensor", "disconnected"}};
    tree.update(parents);
    ASSERT_TRUE((tree.roots == std::vector<std::string>{"disconnected", "map"}));
    ASSERT_TRUE(tree.children.at("gate_frame").at(0) == "target_frame");
    ASSERT_TRUE(tree.frames.at("map").enabled);

    // Hiding a branch hides its descendants and nothing else.
    ASSERT_TRUE(tree.branchSelection("gate_frame") == TfTree::Selection::Shown);
    tree.selectBranch("gate_frame", false);
    ASSERT_TRUE(tree.branchSelection("gate_frame") == TfTree::Selection::Hidden);
    ASSERT_TRUE(!tree.frames.at("target_frame").enabled);
    ASSERT_TRUE(tree.frames.at("talos/base_link").enabled);

    // Individual visibility does not depend on the parent being visible.
    tree.frames.at("target_frame").enabled = true;
    ASSERT_TRUE(!tree.frames.at("gate_frame").enabled);
    ASSERT_TRUE(tree.branchSelection("gate_frame") == TfTree::Selection::Mixed);

    // A parent's own checkbox must not change its descendants, and selecting
    // a whole branch must cover grandchildren even when their rows are collapsed.
    parents["grandchild"] = "target_frame";
    tree.update(parents);
    tree.selectBranch("gate_frame", true);
    tree.frames.at("gate_frame").enabled = false;
    ASSERT_TRUE(tree.frames.at("target_frame").enabled);
    ASSERT_TRUE(tree.frames.at("grandchild").enabled);
    ASSERT_TRUE(tree.branchSelection("gate_frame") == TfTree::Selection::Mixed);
    tree.selectBranch("gate_frame", false);
    ASSERT_TRUE(!tree.frames.at("grandchild").enabled);
    tree.frames.at("gate_frame").enabled = true;
    ASSERT_TRUE(!tree.frames.at("target_frame").enabled);
    ASSERT_TRUE(tree.branchSelection("gate_frame") == TfTree::Selection::Mixed);

    // Reparenting keeps a frame's checkbox; every update() clears `available`.
    tree.frames.at("gate_frame").enabled = false;
    tree.frames.at("gate_frame").available = true;
    parents["gate_frame"] = "disconnected";
    tree.update(parents);
    ASSERT_TRUE(!tree.frames.at("gate_frame").enabled);
    ASSERT_TRUE(!tree.frames.at("gate_frame").available);
    ASSERT_TRUE(tree.children.at("disconnected").at(0) == "gate_frame");

    // After "select none", frames that appear later start hidden.
    tree.selectAll(false);
    parents["new_frame"] = "map";
    tree.update(parents);
    ASSERT_TRUE(!tree.frames.at("new_frame").enabled);

    // A frame that disappears and comes back remembers being hidden.
    tree.selectAll(true);
    tree.frames.at("sensor").enabled = false;
    parents.erase("sensor");
    tree.update(parents);
    parents["sensor"] = "map";
    tree.update(parents);
    ASSERT_TRUE(!tree.frames.at("sensor").enabled);

    // A parent cycle is cut so the frames stay reachable as a root, and branch selection terminates.
    parents["cycle_a"] = "cycle_b";
    parents["cycle_b"] = "cycle_a";
    tree.update(parents);
    tree.selectBranch("cycle_a", false);
    ASSERT_TRUE(!tree.frames.at("cycle_b").enabled);
    ASSERT_TRUE(tree.roots.front() == "cycle_a");
}

// The simulator's static truth frames hang under the truth base link; a real robot has none of them.
TEST(HostTfTree, TruthStaticChildrenAreOptional) {
    TfTree tree;
    std::map<std::string, std::string> parents{
        {"map", ""},
        {"simulator/talos/base_link", "map"},
        {"talos/base_link", "map"},
        {"simulator/talos/origin", "simulator/talos/base_link"},
        {"simulator/talos/ffc_camera_link", "simulator/talos/base_link"},
        {"simulator/talos/ffc_left_camera_optical_frame", "simulator/talos/ffc_camera_link"}};
    tree.update(parents);
    ASSERT_EQ(tree.roots, std::vector<std::string>{"map"});
    ASSERT_EQ(tree.children.at("simulator/talos/base_link").size(), 2u);
    tree.selectBranch("simulator/talos/base_link", false);
    ASSERT_TRUE(tree.branchSelection("simulator/talos/base_link") == TfTree::Selection::Hidden);
    ASSERT_TRUE(!tree.frames.at("simulator/talos/ffc_left_camera_optical_frame").enabled);
    ASSERT_TRUE(tree.frames.at("talos/base_link").enabled);

    // The same tree without the simulator (real robot): still valid, selection state of the rest kept.
    parents.erase("simulator/talos/origin");
    parents.erase("simulator/talos/ffc_camera_link");
    parents.erase("simulator/talos/ffc_left_camera_optical_frame");
    parents.erase("simulator/talos/base_link");
    tree.update(parents);
    ASSERT_EQ(tree.roots, std::vector<std::string>{"map"});
    ASSERT_TRUE(tree.frames.at("talos/base_link").enabled);
}
