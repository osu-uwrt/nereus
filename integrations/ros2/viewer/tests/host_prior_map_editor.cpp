// The prior map editor in the pool view: click to select, drag a prop (its children ride along), keyboard
// nudges, undo / redo, and locked props being click-through. Headless ImGui, a top-down camera, map = world.
#include "prior_map_editor.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <gtest/gtest.h>

using namespace nereus::ros_viewer::host;
namespace pm = prior_map;

namespace {
// A headless ImGui context and an editor on the prior-map fixture, viewed from above the gate.
class EditorTest : public ::testing::Test {
  protected:
    void SetUp() override {
        ImGui::CreateContext();
        auto &io = ImGui::GetIO();
        io.DisplaySize = {800, 600};
        io.DeltaTime = 1.f / 60;
        io.IniFilename = nullptr;
        io.Fonts->Build();
        editor = std::make_unique<PriorMapEditor>(std::filesystem::path(), NEREUS_TEST_FIXTURE);
        editor->setPool({}); // identity pool, origin at the pool corner heading +X: map = world
        editor->setActive(true);
        ASSERT_TRUE(editor->active());

        // Looking straight down at the gate from 12 m.
        const auto gate = pose("gate");
        view.size = {800, 600};
        view.viewProjection = glm::perspective(glm::radians(53.f), 800.f / 600, .05f, 100.f) *
                              glm::lookAt(glm::vec3(gate.x, gate.y, 12), glm::vec3(gate.x, gate.y, gate.z), {0, 1, 0});
        view.hovered = true;
    }

    void TearDown() override {
        editor.reset();
        ImGui::DestroyContext();
    }

    // An object's current pose in the map frame.
    pm::Pose pose(const std::string &name) const {
        return pm::mapPoses(editor->document().objects).at(name);
    }

    // Screen position (pixels, top-left origin) of a world point in the current view.
    glm::vec2 pixel(double x, double y, double z) const {
        const auto clip = view.viewProjection * glm::vec4(float(x), float(y), float(z), 1);
        return view.origin + glm::vec2(clip.x / clip.w * .5f + .5f, .5f - clip.y / clip.w * .5f) * view.size;
    }

    // One frame with the pointer at `cursor` (button down or up), the editor window drawn as well.
    void frame(glm::vec2 cursor, bool down, ImGuiKey key = ImGuiKey_None) {
        auto &io = ImGui::GetIO();
        io.AddMousePosEvent(cursor.x, cursor.y);
        io.AddMouseButtonEvent(0, down);
        if (key != ImGuiKey_None)
            io.AddKeyEvent(key, true);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({800, 600});
        ImGui::Begin("pool view", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs);
        view.pointer.reset();
        editor->input(view);
        editor->shortcuts(true);
        editor->drawOverlay(view, ImGui::GetFont());
        ImGui::End();
        if (drawWindow) {
            editor->drawObjectsWindow("Map objects");
            editor->drawInspectorWindow("Inspector");
            ImGui::Begin("toolbar");
            editor->drawViewTools();
            ImGui::End();
        }
        ImGui::Render();
        if (key != ImGuiKey_None)
            io.AddKeyEvent(key, false);
    }

    // Press and release over three frames.
    void click(glm::vec2 at) {
        frame(at, false);
        frame(at, true);
        frame(at, false);
    }

    std::unique_ptr<PriorMapEditor> editor;
    PriorMapEditor::View view;
    bool drawWindow = false; // also draw the objects / inspector / toolbar windows each frame
};
} // namespace

TEST_F(EditorTest, ClickSelectsAndEmptySpaceClears) {
    const auto gate = pose("gate");
    click(pixel(gate.x, gate.y, gate.z));
    EXPECT_EQ(editor->selected(), "gate");
    click({790, 10}); // far from every prop
    EXPECT_EQ(editor->selected(), "");
}

TEST_F(EditorTest, LockedPropsAreClickThrough) {
    // gate_rescue is locked in the file's editor defaults (a child of the gate assembly)
    ASSERT_TRUE(pm::find(editor->document().objects, "gate_rescue")->locked);
    const auto rescue = pose("gate_rescue");
    click(pixel(rescue.x, rescue.y, rescue.z));
    EXPECT_NE(editor->selected(), "gate_rescue");
}

TEST_F(EditorTest, DraggingMovesThePropAndItsChildren) {
    const auto gate = pose("gate"), rescue = pose("gate_rescue");
    const auto at = pixel(gate.x, gate.y, gate.z);
    click(at);
    ASSERT_EQ(editor->selected(), "gate");
    for (int i = 0; i < 30; ++i) // past the double-click time (a double-click looks at the prop instead)
        frame(at, false);
    // drag 1 m along +X (on the prop's horizontal plane)
    const auto to = pixel(gate.x + 1, gate.y, gate.z);
    frame(at, true);
    frame((at + to) * .5f, true);
    frame(to, true);
    frame(to, false);
    EXPECT_NEAR(pose("gate").x, gate.x + 1, .02);
    EXPECT_NEAR(pose("gate").y, gate.y, .02);
    EXPECT_NEAR(pose("gate").z, gate.z, 1e-9);
    EXPECT_NEAR(pose("gate_rescue").x - rescue.x, pose("gate").x - gate.x, 1e-9); // the child rode along
    EXPECT_TRUE(editor->dirty());

    // one undo step for the whole drag
    frame(to, false, ImGuiKey_None);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, true);
    ImGui::GetIO().AddKeyEvent(ImGuiKey_LeftCtrl, true);
    frame(to, false, ImGuiKey_Z);
    ImGui::GetIO().AddKeyEvent(ImGuiKey_LeftCtrl, false);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, false);
    frame(to, false);
    EXPECT_NEAR(pose("gate").x, gate.x, 1e-9);
    EXPECT_NEAR(pose("gate_rescue").x, rescue.x, 1e-9);
}

// Arrows move 1 cm in x / y, Page Down 1 cm down, Q turns 1 deg; Escape clears the selection.
TEST_F(EditorTest, KeysNudgeInThePoolFrame) {
    const auto gate = pose("gate");
    const auto at = pixel(gate.x, gate.y, gate.z);
    click(at);
    ASSERT_EQ(editor->selected(), "gate");
    frame(at, false, ImGuiKey_RightArrow);
    frame(at, false, ImGuiKey_UpArrow);
    frame(at, false, ImGuiKey_PageDown);
    frame(at, false, ImGuiKey_Q);
    frame(at, false);

    EXPECT_NEAR(pose("gate").x, gate.x + .01, 1e-9);
    EXPECT_NEAR(pose("gate").y, gate.y + .01, 1e-9);
    EXPECT_NEAR(pose("gate").z, gate.z - .01, 1e-9);
    EXPECT_NEAR(pose("gate").yaw, pm::wrapDegrees(gate.yaw + 1), 1e-9);

    frame(at, false, ImGuiKey_Escape);
    EXPECT_EQ(editor->selected(), "");
}

TEST_F(EditorTest, PlanViewDragsAPropWithoutSelectingItFirst) {
    // straight down, orthographic, 20 m top to bottom
    const auto gate = pose("gate"), rescue = pose("gate_rescue");
    const glm::vec3 eye(gate.x, gate.y, 10);
    view.viewProjection = glm::ortho(-10.f * 800 / 600, 10.f * 800 / 600, -10.f, 10.f, .05f, 40.f) *
                          glm::lookAt(eye, eye - glm::vec3(0, 0, 1), {0, 1, 0});
    view.plan = true;
    ASSERT_EQ(editor->selected(), "");
    const auto at = pixel(gate.x, gate.y, gate.z), to = pixel(gate.x + 1, gate.y - .5, gate.z);
    frame(at, false);
    frame(at, true); // pressed on the prop: it moves from here
    frame((at + to) * .5f, true);
    frame(to, true);
    frame(to, false);
    EXPECT_EQ(editor->selected(), "gate");
    EXPECT_NEAR(pose("gate").x, gate.x + 1, .03);
    EXPECT_NEAR(pose("gate").y, gate.y - .5, .03);
    EXPECT_NEAR(pose("gate").z, gate.z, 1e-9); // height untouched
    EXPECT_NEAR(pose("gate_rescue").x - rescue.x, pose("gate").x - gate.x, 1e-9);
}

TEST_F(EditorTest, PlanViewDragFollowsThePointerInARotatedPool) {
    // as the RoboSub scenario: the pool turned -90 deg in the world, the map on its S wall facing +Y
    PriorMapEditor::Pool pool;
    pool.id = "rotated";
    pool.poolToWorld = glm::rotate(glm::translate(glm::mat4(1), {0, 19.5f, 0}), glm::radians(-90.f), {0, 0, 1});
    pool.scenarioOrigin.x = 19.5;
    pool.scenarioOrigin.basePhi = 90;
    editor->setPool(pool);

    const auto gate = pose("gate");
    const auto world = [&](double dx, double dy) { // a map pose offset, in the world
        const auto toWorld =
            pool.poolToWorld * glm::rotate(glm::translate(glm::mat4(1), {19.5f, 0, 0}), glm::radians(90.f), {0, 0, 1});
        return glm::vec3(toWorld * glm::vec4(float(gate.x + dx), float(gate.y + dy), float(gate.z), 1));
    };

    // An orthographic plan view centered on the gate, with screen-up along the pool's +Y.
    const glm::vec3 center = world(0, 0);
    const glm::vec3 eye(center.x, center.y, 10);
    const glm::vec2 poolY(pool.poolToWorld[1]);
    view.viewProjection = glm::ortho(-4.f * 800 / 600, 4.f * 800 / 600, -4.f, 4.f, .05f, 40.f) *
                          glm::lookAt(eye, eye - glm::vec3(0, 0, 1), glm::vec3(poolY, 0));
    view.plan = true;
    const auto onScreen = [&](glm::vec3 p) { return pixel(p.x, p.y, p.z); };

    // Drag in ten steps; the gate must end up where the pointer did, in map coordinates.
    const auto at = onScreen(world(0, 0)), to = onScreen(world(-.4, -1.2));
    frame(at, false);
    frame(at, true);
    for (int i = 1; i <= 10; ++i)
        frame(at + (to - at) * (float(i) / 10), true);
    frame(to, false);
    EXPECT_EQ(editor->selected(), "gate");
    EXPECT_NEAR(pose("gate").x, gate.x - .4, .02);
    EXPECT_NEAR(pose("gate").y, gate.y - 1.2, .02);
    EXPECT_NEAR(pose("gate").z, gate.z, 1e-9);
}

// With a robot-placed map origin, dragging the drawn robot moves the origin (an undoable edit outside the file).
TEST_F(EditorTest, DraggingTheRobotMovesARobotFrameOrigin) {
    pm::Origin robot;
    robot.robot = true;
    robot.x = 10;
    robot.y = 5;
    editor->setOrigin(robot);
    const auto gate = pose("gate");
    ASSERT_TRUE(editor->robotStart());
    EXPECT_NEAR((*editor->robotStart())[3].x, 10, 1e-6); // the robot is drawn on the origin

    // 2D over the robot
    const glm::vec3 eye(10, 5, 10);
    view.viewProjection = glm::ortho(-8.f * 800 / 600, 8.f * 800 / 600, -8.f, 8.f, .05f, 40.f) *
                          glm::lookAt(eye, eye - glm::vec3(0, 0, 1), {0, 1, 0});
    view.plan = true;
    const auto at = pixel(10, 5, 0), to = pixel(11.5, 4, 0);
    frame(at, false);
    frame(at, true);
    for (int i = 1; i <= 10; ++i)
        frame(at + (to - at) * (float(i) / 10), true);
    frame(to, false);
    EXPECT_NEAR(editor->origin().x, 11.5, .03);
    EXPECT_NEAR(editor->origin().y, 4, .03);
    EXPECT_NEAR((*editor->robotStart())[3].x, 11.5, .03);
    EXPECT_NEAR(pose("gate").x, gate.x, 1e-9); // props ride with the map (not pinned to the pool)
    EXPECT_FALSE(editor->dirty());             // the origin is not in the file

    frame(to, false, ImGuiKey_None);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, true);
    frame(to, false, ImGuiKey_Z);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, false);
    frame(to, false);
    EXPECT_NEAR(editor->origin().x, 10, 1e-9); // one undo step
}

TEST_F(EditorTest, AnOriginArrowMovesARobotFrameOriginAlongItsAxis) {
    pm::Origin robot;
    robot.robot = true;
    robot.x = 10;
    robot.y = 5;
    editor->setOrigin(robot);
    const glm::vec3 eye(10, 5, 10);
    view.viewProjection = glm::ortho(-8.f * 800 / 600, 8.f * 800 / 600, -8.f, 8.f, .05f, 40.f) *
                          glm::lookAt(eye, eye - glm::vec3(0, 0, 1), {0, 1, 0});
    view.plan = true;
    // grab the X arrow a meter out (clear of the robot) and pull diagonally: only x follows
    const auto at = pixel(11, 5, 0), to = pixel(12.5, 6, 0);
    frame(at, false);
    frame(at, true);
    for (int i = 1; i <= 10; ++i)
        frame(at + (to - at) * (float(i) / 10), true);
    frame(to, false);
    EXPECT_NEAR(editor->origin().x, 11.5, .03);
    EXPECT_NEAR(editor->origin().y, 5, 1e-6);
    const double x = editor->origin().x;

    // and the Y arrow moves it along y only
    const auto up = pixel(11.5, 6, 0), further = pixel(10.5, 7.5, 0);
    frame(up, false);
    frame(up, true);
    for (int i = 1; i <= 10; ++i)
        frame(up + (further - up) * (float(i) / 10), true);
    frame(further, false);
    EXPECT_NEAR(editor->origin().x, x, 1e-6);
    EXPECT_NEAR(editor->origin().y, 6.5, .03);
}

// A second click right after the first asks the viewer to look at the prop (takeFocus hands it over once).
TEST_F(EditorTest, DoubleClickLooksAtTheProp) {
    const auto gate = pose("gate");
    const auto at = pixel(gate.x, gate.y, gate.z);
    click(at);
    frame(at, true);
    frame(at, false);
    const auto look = editor->takeFocus();
    ASSERT_TRUE(look);
    EXPECT_NEAR(look->x, gate.x, 1e-4);
    EXPECT_NEAR(look->y, gate.y, 1e-4);
    EXPECT_FALSE(editor->takeFocus()); // once
}

// Smoke test: the side windows draw with nothing and then with the gate selected.
TEST_F(EditorTest, WindowDrawsWithAndWithoutASelection) {
    drawWindow = true;
    for (int i = 0; i < 3; ++i)
        frame({790, 590}, false);
    const auto gate = pose("gate");
    click(pixel(gate.x, gate.y, gate.z));
    for (int i = 0; i < 3; ++i)
        frame({790, 590}, false);
    EXPECT_EQ(editor->selected(), "gate");
}

namespace {
// A small Sim course: the gate and the table as tasks (drawn by the scene: no mesh path), the table's bandage as a
// loose object, and one fixed class option.
PriorMapEditor::Course smallCourse() {
    PriorMapEditor::Course course;
    course.label = "Test course";
    course.scenario = "/packs/scenarios/test";
    pm::Object gate, table, bandage;
    gate.name = "gate";
    gate.pose = {5, 1, -.75, 90};
    table.name = "table";
    table.pose = {-3, 2, -1.5, 0};
    bandage.name = "bandage";
    bandage.parent = "table";
    bandage.pose = {.1, .2, .03, 180};
    course.objects = {gate, table, bandage};
    course.frameOf["bandage"] = "bandage";
    course.options = {{"bin_vinyl1_class", "blood", {"blood", "fire"}}};
    for (const char *name : {"gate", "table", "bandage"}) {
        MappingMarker marker;
        marker.frame = std::string(name) + "_frame";
        course.meshes[marker.frame] = marker;
        course.low[marker.frame] = glm::vec3(-.5f);
        course.high[marker.frame] = glm::vec3(.5f);
    }
    course.meshes["bandage_frame"].path = "/meshes/bandage.dae";
    return course;
}

const pm::Object &object(const PriorMapEditor &editor, const std::string &name) {
    return *pm::find(editor.document().objects, name);
}
} // namespace

// The layers swap whole: each keeps its objects and unsaved state; the Sim course's moved task corrects the scene.
TEST_F(EditorTest, SimCourseIsASecondLayer) {
    const auto mapGate = pose("gate");
    editor->setCourse(smallCourse());
    EXPECT_FALSE(editor->editingCourse());
    EXPECT_FALSE(editor->drawsCourse()); // the robot's map hides the simulator's course by default
    EXPECT_TRUE(editor->courseMoves().empty());

    editor->editCourse(true);
    ASSERT_TRUE(editor->editingCourse());
    ASSERT_EQ(editor->document().objects.size(), 3u);
    EXPECT_FALSE(editor->hidesCourse()); // the course being edited is always drawn
    EXPECT_TRUE(editor->drawsCourse());

    // Select the course's gate where it is drawn and nudge it 1 cm along the pool's +X.
    const auto at = pixel(5, 1, -.75);
    click(at);
    ASSERT_EQ(editor->selected(), "gate");
    frame(at, false, ImGuiKey_RightArrow);
    frame(at, false);
    EXPECT_NEAR(object(*editor, "gate").pose.x, 5.01, 1e-9);
    EXPECT_TRUE(editor->courseDirty());
    EXPECT_FALSE(editor->mapDirty());
    const auto moves = editor->courseMoves();
    ASSERT_EQ(moves.count("gate"), 1u);
    EXPECT_NEAR(moves.at("gate")[3].x, .01f, 1e-5f);
    EXPECT_NEAR(moves.at("gate")[3].y, 0.f, 1e-5f);

    // The robot's map is untouched and comes back as it was; the course keeps its edit.
    editor->editCourse(false);
    EXPECT_NEAR(pose("gate").x, mapGate.x, 1e-12);
    EXPECT_TRUE(editor->dirty());
    editor->editCourse(true);
    EXPECT_NEAR(object(*editor, "gate").pose.x, 5.01, 1e-9);

    // A new scenario's course replaces it (the scene is rebuilt from it: no corrections).
    editor->setCourse(smallCourse());
    EXPECT_TRUE(editor->courseMoves().empty());
    EXPECT_FALSE(editor->courseDirty());
}

// Save hands the host set-course JSON with what changed; the host's report ends the save.
TEST_F(EditorTest, SimCourseSaveSendsTheEdit) {
    editor->setCourse(smallCourse());
    std::filesystem::path savedTo;
    std::string sent;
    editor->setCourseSaver([&](const std::filesystem::path &scenario, const std::string &edit) {
        savedTo = scenario;
        sent = edit;
    });
    editor->editCourse(true);
    EXPECT_EQ(editor->courseEdit(), "");

    const auto at = pixel(5, 1, -.75);
    click(at);
    frame(at, false, ImGuiKey_RightArrow);
    frame(at, false);
    const std::string expected = "{\"task_placements\": [{\"task\": \"gate\", \"position_m\": [5.01, 1, -0.75], "
                                 "\"yaw_deg\": 90}], \"task_frames\": [], \"run_options\": {}}";
    EXPECT_EQ(editor->courseEdit(), expected);

    editor->save();
    EXPECT_TRUE(editor->savingCourse());
    EXPECT_EQ(savedTo, "/packs/scenarios/test");
    EXPECT_EQ(sent, expected);

    editor->courseSaved("the scenario does not resolve");
    EXPECT_FALSE(editor->savingCourse());
    EXPECT_TRUE(editor->courseDirty());
    EXPECT_NE(editor->message().find("Not saved: the scenario does not resolve"), std::string::npos);

    editor->save();
    editor->courseSaved("");
    EXPECT_FALSE(editor->courseDirty());
    EXPECT_EQ(editor->courseEdit(), "");
    EXPECT_EQ(editor->courseMoves().count("gate"), 1u); // until the simulator's new scene arrives
}

// The copy buttons: linked tasks, their loose objects and the classes, each way, one undo step each.
TEST_F(EditorTest, CopiesBetweenTheRobotsMapAndTheSimCourse) {
    editor->setCourse(smallCourse());
    editor->setCourseLinks({{{"gate", "gate"}, {"table", "table"}}, {"gate"}}); // the gate floats

    // Robot's map -> Sim course (map = world here).
    const auto mapGate = pose("gate"), mapTable = pose("table"), mapBandage = pose("bandage");
    const auto mapClass = pm::find(editor->document().objects, "bin_vinyl1")->cls;
    editor->editCourse(true);
    editor->copyFromOtherLayer();
    const auto &gate = object(*editor, "gate").pose;
    EXPECT_NEAR(gate.x, mapGate.x, 1e-5);
    EXPECT_NEAR(gate.y, mapGate.y, 1e-5);
    EXPECT_NEAR(gate.z, -.75, 1e-12); // floating: the course keeps its height
    EXPECT_NEAR(pm::wrapDegrees(gate.yaw - mapGate.yaw), 0, 1e-3);
    const auto onTable = pm::decompose(mapTable, mapBandage);
    EXPECT_NEAR(object(*editor, "bandage").pose.x, onTable.x, 1e-9);
    EXPECT_NEAR(object(*editor, "bandage").pose.y, onTable.y, 1e-9);
    EXPECT_NE(editor->courseEdit().find("\"bin_vinyl1_class\": \"" + mapClass + "\""), std::string::npos);
    editor->editCourse(false);

    // Sim course -> robot's map, from a course laid out elsewhere.
    auto moved = smallCourse();
    moved.objects[1].pose = {4, -6, -2, 30}; // the table
    moved.options[0].value = mapClass == "fire" ? "blood" : "fire";
    editor->setCourse(moved);
    editor->copyFromOtherLayer();
    EXPECT_NEAR(pose("table").x, 4, 1e-5);
    EXPECT_NEAR(pose("table").y, -6, 1e-5);
    EXPECT_NEAR(pose("table").z, -2, 1e-5);
    EXPECT_NEAR(pm::wrapDegrees(pose("table").yaw - 30), 0, 1e-3);
    const auto bandage = pm::compose(pose("table"), {.1, .2, .03, 180});
    EXPECT_NEAR(pose("bandage").x, bandage.x, 1e-9);
    EXPECT_NEAR(pose("bandage").y, bandage.y, 1e-9);
    EXPECT_NEAR(pose("gate").z, mapGate.z, 1e-9); // floating: the map keeps its height
    EXPECT_EQ(pm::find(editor->document().objects, "bin_vinyl1")->cls, moved.options[0].value);
    EXPECT_TRUE(editor->mapDirty());

    // One undo puts the map back.
    frame({790, 590}, false, ImGuiKey_None);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, true);
    frame({400, 300}, false, ImGuiKey_Z);
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Ctrl, false);
    frame({400, 300}, false);
    EXPECT_NEAR(pose("table").x, mapTable.x, 1e-9);
}

// The windows draw in the Sim course with and without a selection (its own bar and inspector).
TEST_F(EditorTest, SimCourseWindowsDraw) {
    drawWindow = true;
    editor->setCourse(smallCourse());
    editor->editCourse(true);
    for (int i = 0; i < 3; ++i)
        frame({790, 590}, false);
    click(pixel(-3, 2, -1.5));
    EXPECT_EQ(editor->selected(), "table");
    for (int i = 0; i < 3; ++i)
        frame({790, 590}, false);
}
