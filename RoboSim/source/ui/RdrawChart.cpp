#include <pch.h>
#include "RdrawChart.h"
namespace nui
{
    void RdrawChart::render()
    {
        if (robinit->get_settings("theme") == "Dark"){FontCol = IM_COL32(250, 230, 0, 255);}
        else {FontCol = IM_COL32(125, 50, 125, 255);}
        static std::unique_ptr<nui::RdrawChart> drawChart = std::make_unique<nui::RdrawChart>();
        
        ImGui::Begin("Chart");
        ImGui::BeginGroup();
        ImVec2 windowSize = ImGui::GetWindowSize();

        ImGui::BeginChild("BarLineChart", ImVec2(windowSize.x, windowSize.y * 0.8f)); 
        drawChart->DrawBarLineChart(); ImGui::EndChild(); ImGui::SameLine();
        ImGui::EndGroup(); ImGui::End();

    }

    RdrawChart::RdrawChart()
    { 
    }

    RdrawChart::~RdrawChart() {}




    void DrawRect3D(ImVec2 topLeft, ImVec2 bottomRight, ImU32 color, float thickness = 2.0f, float shadowOffset = 6.0f)
    {
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        // Draw shadow (offset down and right)
        ImVec2 shadowTL = ImVec2(topLeft.x + shadowOffset, topLeft.y + shadowOffset);
        ImVec2 shadowBR = ImVec2(bottomRight.x + shadowOffset, bottomRight.y + shadowOffset);
        drawList->AddRectFilled(shadowTL, shadowBR, IM_COL32(55, 0, 0, 100));  // Semi-transparent black shadow

        // Draw the main rectangle
        drawList->AddRectFilled(topLeft, bottomRight, color);

        // Draw the top and left edge (highlight)
        ImVec2 topRight = ImVec2(bottomRight.x, topLeft.y);
        ImVec2 bottomLeft = ImVec2(topLeft.x, bottomRight.y);
        //drawList->AddLine(topLeft, topRight, IM_COL32(255, 0, 120, 150), thickness);  // Highlight on top
        //drawList->AddLine(topLeft, bottomLeft, IM_COL32(255, 0, 120, 150), thickness); // Highlight on left

        // Draw the bottom and right edge (darken)
        drawList->AddLine(bottomLeft, bottomRight, IM_COL32(55, 0, 0, 150), thickness);  // Darken on bottom
        drawList->AddLine(topRight, bottomRight, IM_COL32(55, 0, 0, 150), thickness);    // Darken on right
    }
    void DrawRect3D(ImVec2 topLeft, ImVec2 bottomRight, float thickness = 2.0f, float shadowOffset = 6.0f)
    {
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        // Draw shadow (offset down and right)
        ImVec2 shadowTL = ImVec2(topLeft.x + shadowOffset, topLeft.y + shadowOffset);
        ImVec2 shadowBR = ImVec2(bottomRight.x + shadowOffset, bottomRight.y + shadowOffset);
        drawList->AddRectFilled(shadowTL, shadowBR, IM_COL32(80, 80, 80, 250));  // Semi-transparent black shadow

        // Draw the main rectangle with gradient
        ImU32 topColor = IM_COL32(255, 0, 0, 255);    // Lighter color at the top
        ImU32 bottomColor = IM_COL32(0, 250, 5, 255);     // Darker color at the bottom
        drawList->AddRectFilledMultiColor(topLeft, bottomRight, topColor, topColor, bottomColor, bottomColor);

        // Draw the top and left edge (highlight)
        ImVec2 topRight = ImVec2(bottomRight.x, topLeft.y);
        ImVec2 bottomLeft = ImVec2(topLeft.x, bottomRight.y);
        drawList->AddLine(topLeft, topRight, IM_COL32(255, 0, 120, 150), thickness);  // Highlight on top
        drawList->AddLine(topLeft, bottomLeft, IM_COL32(255, 0, 120, 150), thickness); // Highlight on left

        // Draw the bottom and right edge (darken)
        drawList->AddLine(bottomLeft, bottomRight, IM_COL32(55, 0, 0, 150), thickness);  // Darken on bottom
        drawList->AddLine(topRight, bottomRight, IM_COL32(55, 0, 0, 150), thickness);    // Darken on right
    }

    


    void RdrawChart::DrawBarLineChart()
    {        
        ImVec2 windowSize = ImGui::GetContentRegionAvail(); // Get the available content region size

        float maxChartHeight = windowSize.y - 20; // Leave some space for the x-axis
        float maxChartWidth = windowSize.x - 20; // Leave some space for the y-axis
        float barWidth = maxChartWidth / Chartdata.size() - 10; // Calculate the width of each bar, including spacing
        float pointRadius = 3.0f;
        ImVec2 origin(ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + maxChartHeight);

        // Draw the x and y axes
        ImVec2 xAxisEnd = ImVec2(origin.x + maxChartWidth, origin.y);
        ImVec2 yAxisEnd = ImVec2(origin.x, origin.y - maxChartHeight);
        ImGui::GetWindowDrawList()->AddLine(origin, xAxisEnd, FontCol); // X-axis
        ImGui::GetWindowDrawList()->AddLine(origin, yAxisEnd, FontCol); // Y-axis

        int i = 0;
        std::vector<ImVec2> points;
        for (const auto& [content,value] : Chartdata)
        {
            // Calculate bar dimensions
            float barHeight = (value / 100.0f) * maxChartHeight;
            ImVec2 barMin(origin.x + i * (barWidth + 10), origin.y - barHeight);
            ImVec2 barMax(barMin.x + barWidth, origin.y);
            ImU32 color = ImGui::GetColorU32(ImVec4((float)i / Chartdata.size(), 0.6f, 0.6f, 1.0f));
            DrawRect3D(barMin, barMax);
            // Draw the x-axis labels
            ImVec2 labelPos = ImVec2(barMin.x + barWidth / 2, origin.y + 5);
            ImGui::GetWindowDrawList()->AddText(labelPos, FontCol, "");

            // Calculate line chart points
            float x = barMin.x + barWidth / 2;
            float y = origin.y - (value / 100.0f) * maxChartHeight;
            points.emplace_back(x, y);

            // Check if the mouse is hovering over the bar
            ImVec2 mousePos = ImGui::GetMousePos();
            if (mousePos.x >= barMin.x && mousePos.x <= barMax.x && mousePos.y >= barMin.y && mousePos.y <= barMax.y)
            {
                ImGui::SetTooltip("\nData: %.1f\nContent: %s",value, content);
            }

            ++i;
        }

        // Draw lines
        for (int j = 0; j < points.size() - 1; ++j)
        {
            ImGui::GetWindowDrawList()->AddLine(points[j], points[j + 1], ImGui::GetColorU32(ImVec4(0.0f, 0.6f, 1.0f, 1.0f)));
        }

        // Draw points and check if mouse is hovering over a point
        i = 0;
        for (const auto& [content, value] : Chartdata)
        {
            float x = points[i].x;
            float y = points[i].y;
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(x, y), pointRadius, ImGui::GetColorU32(ImVec4(1.0f, 0.6f, 0.6f, 1.0f)));
            ++i;
        }
    }


}
