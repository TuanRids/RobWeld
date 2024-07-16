#include <pch.h>
#include "RdrawChart.h"
#include <cmath>
#include <GLFW/glfw3.h>

namespace nui
{
    void RdrawChart::render()
    {
        
        static std::unique_ptr<nui::RdrawChart> drawChart = std::make_unique<nui::RdrawChart>();
        static std::map<std::string, std::pair<float, std::string>> data = {
            {"Weld A", {60.2f , " This section involves the initial welding process to join metal parts together."}},
            {"Weld B", {75.5f , " This section includes the quality inspection and verification of the welded joints."}},
            {"Weld C", {85.4f , " This section covers the secondary welding for reinforcing the initial joints."}},
            {"Weld D", {93.0f , " This section focuses on cooling and setting the welded parts to ensure stability."}},
            {"Weld E", {66.3f , " This section involves post-welding treatments such as grinding and smoothing the surfaces."}},
            {"Weld F", {96.7f , " This section is dedicated to the final inspection and certification of the welded components."}}
        };
        ImGui::Begin("Chart");
        ImGui::BeginGroup();
        ImVec2 windowSize = ImGui::GetWindowSize();
        /*ImGui::BeginChild("BarChart", ImVec2(0, windowSize.y * 0.3f));
        drawChart->DrawBarChart(data); ImGui::EndChild();*/
        ImGui::BeginChild("LineChart", ImVec2(windowSize.x*0.7f, windowSize.y * 0.4f));
        drawChart->DrawLineChart(data); ImGui::EndChild(); ImGui::SameLine();
        ImGui::BeginChild("PieChart", ImVec2(windowSize.x * 0.5f, windowSize.y * 0.4f));
        drawChart->DrawPieChart(data); ImGui::EndChild();
        ImGui::BeginChild("BarLineChart", ImVec2(windowSize.x, windowSize.y * 0.4f)); 
        drawChart->DrawBarLineChart(data); ImGui::EndChild(); ImGui::SameLine();
        ImGui::EndGroup(); ImGui::End();
    }

    RdrawChart::RdrawChart() {}

    RdrawChart::~RdrawChart() {}

    void RdrawChart::DrawTable(const std::vector<std::vector<float>>& data)
    {
        ImGui::Begin("Table");
        for (const auto& row : data)
        {
            for (const auto& cell : row)
            {
                ImGui::Text("%f", cell);
                ImGui::SameLine();
            }
            ImGui::NewLine();
        }
        ImGui::End();
    }

    void RdrawChart::DrawBarChart(const std::map<std::string, std::pair<float, std::string>>& data)
    {
        
            ImVec2 windowSize = ImGui::GetContentRegionAvail(); // Get the available content region size

            float maxBarHeight = windowSize.y - 40; // Leave some space for the x-axis
            float totalBarsWidth = windowSize.x - 20; // Leave some space for the y-axis
            float barWidth = totalBarsWidth / data.size() - 10; // Calculate the width of each bar, including spacing

            ImVec2 origin(ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + maxBarHeight);

            // Draw the x and y axes
            ImVec2 xAxisEnd = ImVec2(origin.x + totalBarsWidth, origin.y);
            ImVec2 yAxisEnd = ImVec2(origin.x, origin.y - maxBarHeight);
            ImGui::GetWindowDrawList()->AddLine(origin, xAxisEnd, IM_COL32(125, 0, 125, 255)); // X-axis
            ImGui::GetWindowDrawList()->AddLine(origin, yAxisEnd, IM_COL32(125, 0, 125, 255)); // Y-axis

            int i = 0;
            for (const auto& [key, value] : data)
            {
                float barHeight = (value.first / 100.0f) * maxBarHeight;
                ImVec2 barMin(origin.x + i * (barWidth + 10), origin.y - barHeight);
                ImVec2 barMax(barMin.x + barWidth, origin.y);
                ImU32 color = ImGui::GetColorU32(ImVec4((float)i / data.size(), 0.6f, 0.6f, 1.0f));
                ImGui::GetWindowDrawList()->AddRectFilled(barMin, barMax, color);

                // Draw the x-axis labels
                ImVec2 labelPos = ImVec2(barMin.x + barWidth / 2, origin.y + 5);
                ImGui::GetWindowDrawList()->AddText(labelPos, IM_COL32(125, 0, 125, 255), key.c_str());

                // Check if the mouse is hovering over the bar
                ImVec2 mousePos = ImGui::GetMousePos();
                if (mousePos.x >= barMin.x && mousePos.x <= barMax.x && mousePos.y >= barMin.y && mousePos.y <= barMax.y)
                {
                    ImGui::SetTooltip("%s\nData: %.1f\nContent: %s", key.c_str(), value.first, value.second.c_str());
                }
                ++i;
            }

        
    }


    void RdrawChart::DrawLineChart(const std::map<std::string, std::pair<float, std::string>>& data)
    {
       
            ImVec2 windowSize = ImGui::GetContentRegionAvail(); // Get the available content region size

            float maxChartHeight = windowSize.y - 40; // Leave some space for the x-axis
            float maxChartWidth = windowSize.x - 20; // Leave some space for the y-axis
            float pointRadius = 3.0f;
            ImVec2 origin(ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + maxChartHeight);

            // Draw the x and y axes
            ImVec2 xAxisEnd = ImVec2(origin.x + maxChartWidth, origin.y);
            ImVec2 yAxisEnd = ImVec2(origin.x, origin.y - maxChartHeight);
            ImGui::GetWindowDrawList()->AddLine(origin, xAxisEnd, IM_COL32(125, 0, 125, 255)); // X-axis
            ImGui::GetWindowDrawList()->AddLine(origin, yAxisEnd, IM_COL32(125, 0, 125, 255)); // Y-axis

            int i = 0;
            std::vector<ImVec2> points;
            for (const auto& [key, value] : data)
            {
                float x = origin.x + i * (maxChartWidth / (data.size() - 1));
                float y = origin.y - (value.first / 100.0f) * maxChartHeight;
                points.emplace_back(x, y);

                // Draw the x-axis labels
                ImVec2 labelPos = ImVec2(x, origin.y + 5);
                ImGui::GetWindowDrawList()->AddText(labelPos, IM_COL32(125, 0, 125, 255), key.c_str());

                ++i;
            }

            // Draw lines
            for (int j = 0; j < points.size() - 1; ++j)
            {
                ImGui::GetWindowDrawList()->AddLine(points[j], points[j + 1], ImGui::GetColorU32(ImVec4(0.0f, 0.6f, 1.0f, 1.0f)));
            }

            // Draw points and check if mouse is hovering over a point
            i = 0;
            for (const auto& [key, value] : data)
            {
                float x = points[i].x;
                float y = points[i].y;
                ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(x, y), pointRadius, ImGui::GetColorU32(ImVec4(1.0f, 0.6f, 0.6f, 1.0f)));

                ImVec2 mousePos = ImGui::GetMousePos();
                float dx = mousePos.x - x;
                float dy = mousePos.y - y;
                float distance = sqrtf(dx * dx + dy * dy);
                if (distance < pointRadius)
                {
                    ImGui::SetTooltip("%s\nData: %.1f\nContent: %s", key.c_str(), value.first, value.second.c_str());
                }
                ++i;
            }

    }

    void RdrawChart::DrawBarLineChart(const std::map<std::string, std::pair<float, std::string>>& data)
    {
        
            ImVec2 windowSize = ImGui::GetContentRegionAvail(); // Get the available content region size

            float maxChartHeight = windowSize.y - 40; // Leave some space for the x-axis
            float maxChartWidth = windowSize.x - 20; // Leave some space for the y-axis
            float barWidth = maxChartWidth / data.size() - 10; // Calculate the width of each bar, including spacing
            float pointRadius = 3.0f;
            ImVec2 origin(ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + maxChartHeight);

            // Draw the x and y axes
            ImVec2 xAxisEnd = ImVec2(origin.x + maxChartWidth, origin.y);
            ImVec2 yAxisEnd = ImVec2(origin.x, origin.y - maxChartHeight);
            ImGui::GetWindowDrawList()->AddLine(origin, xAxisEnd, IM_COL32(125, 0, 125, 255)); // X-axis
            ImGui::GetWindowDrawList()->AddLine(origin, yAxisEnd, IM_COL32(125, 0, 125, 255)); // Y-axis

            int i = 0;
            std::vector<ImVec2> points;
            for (const auto& [key, value] : data)
            {
                // Calculate bar dimensions
                float barHeight = (value.first / 100.0f) * maxChartHeight;
                ImVec2 barMin(origin.x + i * (barWidth + 10), origin.y - barHeight);
                ImVec2 barMax(barMin.x + barWidth, origin.y);
                ImU32 color = ImGui::GetColorU32(ImVec4((float)i / data.size(), 0.6f, 0.6f, 1.0f));
                ImGui::GetWindowDrawList()->AddRectFilled(barMin, barMax, color);

                // Draw the x-axis labels
                ImVec2 labelPos = ImVec2(barMin.x + barWidth / 2, origin.y + 5);
                ImGui::GetWindowDrawList()->AddText(labelPos, IM_COL32(125, 0, 125, 255), key.c_str());

                // Calculate line chart points
                float x = barMin.x + barWidth / 2;
                float y = origin.y - (value.first / 100.0f) * maxChartHeight;
                points.emplace_back(x, y);

                // Check if the mouse is hovering over the bar
                ImVec2 mousePos = ImGui::GetMousePos();
                if (mousePos.x >= barMin.x && mousePos.x <= barMax.x && mousePos.y >= barMin.y && mousePos.y <= barMax.y)
                {
                    ImGui::SetTooltip("Key: %s\nData: %.1f\nContent: %s", key.c_str(), value.first, value.second.c_str());
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
            for (const auto& [key, value] : data)
            {
                float x = points[i].x;
                float y = points[i].y;
                ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(x, y), pointRadius, ImGui::GetColorU32(ImVec4(1.0f, 0.6f, 0.6f, 1.0f)));

                ImVec2 mousePos = ImGui::GetMousePos();
                float dx = mousePos.x - x;
                float dy = mousePos.y - y;
                float distance = sqrtf(dx * dx + dy * dy);
                if (distance < pointRadius)
                {
                    ImGui::SetTooltip("Key: %s\nData: %.1f\nContent: %s", key.c_str(), value.first, value.second.c_str());
                }
                ++i;
            }

    }

    void RdrawChart::DrawBarLineQc(const std::map<std::string, std::pair<float, std::string>>& data)
    {


    }


    void RdrawChart::DrawPieChart(const std::map<std::string, std::pair<float, std::string>>& data)
    {
        
            ImVec2 windowSize = ImGui::GetContentRegionAvail(); // Get the available content region size
            float radius = (std::min(windowSize.x, windowSize.y) - 40) / 2; // Ensure the pie chart fits within the available space
            ImVec2 center(ImGui::GetCursorScreenPos().x + radius, ImGui::GetCursorScreenPos().y + radius);

            // Calculate total sum
            float total = 0.0f;
            for (const auto& [key, value] : data)
                total += value.first;

            float angle = 0.0f;
            int i = 0;
            for (const auto& [key, value] : data)
            {
                float percentage = (value.first / total) * 360.0f;
                float startAngle = angle * 3.1415926535f / 180.0f;
                angle += percentage;
                float endAngle = angle * 3.1415926535f / 180.0f;

                // Draw arc
                ImU32 color = ImGui::GetColorU32(ImVec4((float)i / data.size(), 0.6f, 0.6f, 1.0f));
                ImGui::GetWindowDrawList()->PathLineTo(center);
                for (int j = 0; j <= 32; ++j)
                {
                    float theta = startAngle + j * (endAngle - startAngle) / 32;
                    ImVec2 point(center.x + cosf(theta) * radius, center.y + sinf(theta) * radius);
                    ImGui::GetWindowDrawList()->PathLineTo(point);
                }
                ImGui::GetWindowDrawList()->PathFillConvex(color);

                // Check if mouse is hovering over the arc
                ImVec2 mousePos = ImGui::GetMousePos();
                float dx = mousePos.x - center.x;
                float dy = mousePos.y - center.y;
                float distance = sqrtf(dx * dx + dy * dy);
                float hoverAngle = atan2f(dy, dx);
                if (hoverAngle < 0) hoverAngle += 2 * 3.1415926535f;

                if (distance < radius && hoverAngle >= startAngle && hoverAngle <= endAngle)
                {
                    ImGui::SetTooltip("%s\nData: %.1f\nContent: %s", key.c_str(), value.first, value.second.c_str());
                }
                ++i;
            }

    }

}
