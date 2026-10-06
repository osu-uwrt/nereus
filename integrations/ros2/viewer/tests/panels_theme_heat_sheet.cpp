// The Heat sheet pair (the default theme and its dark twin): shipped tabular-figure fonts, the legibility floor on
// sheets and on the board, and the board's own palette inside beginSurface.
#include "nereus/ros_viewer/theme.hpp"
#include <cassert>
#include <cmath>
#include <filesystem>
#include <imgui.h>
#include <iostream>
#include <string>

using namespace nereus::ros_viewer;

int main(int argc, char **argv) {
    assert(argc > 1); // content/viewer/fonts
    const std::filesystem::path fonts = argv[1];
    assert(loadThemes(NEREUS_VIEWER_THEMES).empty());
    assert(themes().front().id == "heat-sheet"); // the default
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.DisplaySize = {800, 600};
    for (const char *id : {"heat-sheet", "timing-board"}) {
        assert(applyTheme(id));
        const auto &theme = currentThemeInfo();
        // the fonts ship with the viewer, and every digit has one advance (columns of figures line up)
        for (const auto &file : {theme.fontRegular, theme.fontStrong, theme.fontFigures}) {
            assert(!file.empty() && std::filesystem::exists(fonts / file));
            ImFontAtlas atlas;
            ImFont *font = atlas.AddFontFromFileTTF((fonts / file).c_str(), 20);
            assert(font && atlas.Build());
            const float zero = font->FindGlyph('0')->AdvanceX;
            for (char digit = '1'; digit <= '9'; ++digit)
                assert(std::abs(font->FindGlyph(digit)->AdvanceX - zero) < .01f);
        }
        assert(ruledTheme() && ImGui::GetStyle().FrameRounding == 2 && ImGui::GetStyle().FrameBorderSize == 1);
        const auto &c = ImGui::GetStyle().Colors;
        const Palette sheet = palette();
        assert(contrastRatio(sheet.text, c[ImGuiCol_WindowBg]) >= 12);
        assert(contrastRatio(sheet.muted, c[ImGuiCol_WindowBg]) >= 5);
        assert(contrastRatio(sheet.accent, c[ImGuiCol_WindowBg]) >= 5);
        assert(contrastRatio(sheet.changeText, sheet.change) >= 7); // a lit figure (the running clock) reads
        assert(c[ImGuiCol_TableHeaderBg].w == 0);                   // ruled heads, no header fill
        // the board: light type on the dark bar, its own colours inside the surface, the sheet's after
        beginSurface(Surface::Board);
        const Palette board = palette();
        assert(contrastRatio(board.text, board.bar) >= 12 && contrastRatio(board.muted, board.bar) >= 5);
        assert(contrastRatio(board.robotEnabled, board.bar) >= 5 &&
               contrastRatio(board.dangerText, board.danger) >= 4.5f);
        assert(contrastRatio(ImGui::GetStyleColorVec4(ImGuiCol_Text), board.bar) >= 12);
        beginSurface(Surface::Sheet); // a dropdown on the board
        assert(contrastRatio(palette().text, c[ImGuiCol_WindowBg]) >= 12);
        endSurface();
        endSurface();
        assert(palette().text.x == sheet.text.x && ImGui::GetStyleColorVec4(ImGuiCol_Text).x == sheet.text.x);
    }
    ImGui::DestroyContext();
    std::cout << "PASS: Heat sheet pair (tabular fonts, legibility, board surface)\n";
}
