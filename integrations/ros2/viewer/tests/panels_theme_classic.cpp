// The Classic theme: Qt's Windows-style palette (as RViz looks on the desktop) with the Ubuntu font.
#include "nereus/ros_viewer/theme.hpp"
#include <cassert>
#include <cmath>
#include <imgui.h>
#include <iostream>

using namespace nereus::ros_viewer;

int main() {
    // The shipped theme files load and Classic asks for Ubuntu 11 pt.
    assert(loadThemes(NEREUS_VIEWER_THEMES).empty());
    const Theme *classic = nullptr;
    for (const auto &theme : themes())
        if (theme.id == "classic")
            classic = &theme;
    assert(classic && classic->fontFamily == "Ubuntu" && classic->fontPoints == 11);

    ImGui::CreateContext();
    assert(applyTheme("classic") && currentThemeInfo().fontFamily == "Ubuntu");
    const auto &c = ImGui::GetStyle().Colors;
    // RGB match within 1 % per channel (alpha ignored).
    const auto near = [](ImVec4 a, float r, float g, float b) {
        return std::abs(a.x - r) < .01f && std::abs(a.y - g) < .01f && std::abs(a.z - b) < .01f;
    };

    // Win 2000 gray, white fields, Qt::gray title strips, navy highlight, black text.
    assert(near(c[ImGuiCol_WindowBg], 212 / 255.f, 208 / 255.f, 200 / 255.f) && near(c[ImGuiCol_FrameBg], 1, 1, 1));
    assert(near(c[ImGuiCol_TitleBg], 160 / 255.f, 160 / 255.f, 164 / 255.f) && near(c[ImGuiCol_Text], 0, 0, 0));
    assert(near(palette().accent, 0, 0, .5f) && near(palette().activeText, 1, 1, 1));
    assert(near(c[ImGuiCol_Border], 1, 1, 1) && near(c[ImGuiCol_BorderShadow], 0, 0, 0)); // raised bevels
    assert(ImGui::GetStyle().FrameRounding == 0 && ImGui::GetStyle().FrameBorderSize == 1);
    assert(applyTheme("abyss") && currentThemeInfo().fontFamily.empty()); // the others keep the viewer's font

    ImGui::DestroyContext();
    std::cout << "PASS: Classic theme (Qt Windows-style palette, bevels, Ubuntu font)\n";
}
