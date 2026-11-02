#include "Objects/Simulation/simulation.h"

#include <windows.h>
#include <windowsx.h>

#include <algorithm>
#include <cmath>
#include <cwchar>
#include <stdexcept>
#include <string>

using namespace traffic;

namespace {

constexpr int sidebar = 310;
constexpr int margin = 18;
constexpr int controlWidth = sidebar - 2 * margin;
constexpr int idSelect = 101;
constexpr int idRoad = 102;
constexpr int idSpawn = 103;
constexpr int idRun = 104;
constexpr int idStep = 105;
constexpr int idReset = 106;
constexpr int idDelete = 107;
constexpr int idClear = 108;
constexpr int idRate = 109;
constexpr int idApply = 110;
constexpr int idForwardLanes = 111;
constexpr int idBackwardLanes = 112;
constexpr int idSpeedLimit = 113;
constexpr int idMinInitialSpeed = 114;
constexpr int idMaxInitialSpeed = 115;
constexpr double snapDistance = 18.0;

enum class Tool { select, road, spawn };
enum class DragMode { none, newRoad, moveRoad, start, end };

Simulation simulation;
Tool tool = Tool::road;
bool running = false;
bool dragging = false;
DragMode dragMode = DragMode::none;
bool editStarted = false;
Point dragStart, dragEnd;
Point pointerStart, originalStart, originalEnd;
int selectedRoad = -1;
int selectedSpawn = -1;
HWND mainWindow = nullptr;
HWND runButton = nullptr;
HWND rateEdit = nullptr;
HWND minInitialSpeedEdit = nullptr;
HWND maxInitialSpeedEdit = nullptr;
HWND forwardEdit = nullptr;
HWND backwardEdit = nullptr;
HWND speedLimitEdit = nullptr;
HFONT uiFont = nullptr;
ULONGLONG lastFrame = 0;
double elapsed = 0;
std::wstring notice = L"Нарисуйте дорогу: зажмите и протяните мышь.";

Point mousePoint(LPARAM lparam) {
    return {static_cast<double>(GET_X_LPARAM(lparam) - sidebar),
            static_cast<double>(GET_Y_LPARAM(lparam))};
}

bool onCanvas(POINT p) {
    RECT bounds;
    GetClientRect(mainWindow, &bounds);
    return p.x >= sidebar && p.x < bounds.right && p.y >= 0 && p.y < bounds.bottom;
}

double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }

Point snappedEndpoint(Point p, int excludedRoad = -1, bool* snapped = nullptr) {
    Point result = p;
    double best = snapDistance;
    bool found = false;
    for (const auto& road : simulation.roads()) {
        if (road->id() == excludedRoad) continue;
        for (Point candidate : {road->start(), road->end()}) {
            const double d = distance(p, candidate);
            if (d < best) { best = d; result = candidate; found = true; }
        }
    }
    if (snapped) *snapped = found;
    return result;
}

Road* roadHandleAt(Point p, bool& start) {
    Road* result = nullptr;
    double best = 11;
    for (const auto& road : simulation.roads()) {
        for (bool atStart : {true, false}) {
            const double d = distance(p, atStart ? road->start() : road->end());
            if (d < best || (d == best && road->id() == selectedRoad)) {
                best = d;
                result = road.get();
                start = atStart;
            }
        }
    }
    return result;
}

void updateRoadDrag(Point pointer) {
    Road* road = nullptr;
    for (const auto& candidate : simulation.roads())
        if (candidate->id() == selectedRoad) road = candidate.get();
    if (!road) return;
    if (distance(pointer, pointerStart) < 2 && !editStarted) return;
    if (!editStarted) {
        simulation.reset();
        editStarted = true;
    }
    Point start = originalStart, end = originalEnd;
    if (dragMode == DragMode::start) start = snappedEndpoint(pointer, road->id());
    if (dragMode == DragMode::end) end = snappedEndpoint(pointer, road->id());
    if (dragMode == DragMode::moveRoad) {
        const double dx = pointer.x - pointerStart.x;
        const double dy = pointer.y - pointerStart.y;
        start = {start.x + dx, start.y + dy};
        end = {end.x + dx, end.y + dy};
        bool startSnapped = false, endSnapped = false;
        Point candidateStart = snappedEndpoint(start, road->id(), &startSnapped);
        Point candidateEnd = snappedEndpoint(end, road->id(), &endSnapped);
        if (startSnapped && (!endSnapped ||
            distance(start, candidateStart) <= distance(end, candidateEnd))) {
            end.x += candidateStart.x - start.x;
            end.y += candidateStart.y - start.y;
            start = candidateStart;
        } else if (endSnapped) {
            start.x += candidateEnd.x - end.x;
            start.y += candidateEnd.y - end.y;
            end = candidateEnd;
        }
    }
    try { road->setEndpoints(start, end); }
    catch (const std::invalid_argument&) { return; }
    if (selectedSpawn >= 0) selectedSpawn = -1;
    InvalidateRect(mainWindow, nullptr, FALSE);
}

double distanceToRoad(Point p, const Road& road) {
    double best = 1e9;
    for (Direction direction : {Direction::forward, Direction::backward}) {
        for (int lane = 0; lane < road.laneCount(direction); ++lane) {
            const Point a = road.lanePointAt(0, direction, lane);
            const Point b = road.lanePointAt(road.length(), direction, lane);
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) /
                                        (dx * dx + dy * dy), 0.0, 1.0);
            best = std::min(best, distance(p, {a.x + t * dx, a.y + t * dy}));
        }
    }
    return best;
}

Road* roadById(int id) {
    for (const auto& road : simulation.roads()) if (road->id() == id) return road.get();
    return nullptr;
}

SpawnPoint* spawnById(int id) {
    for (const auto& point : simulation.spawnPoints()) if (point->id() == id) return point.get();
    return nullptr;
}

bool roadParameters(int& forward, int& backward, double& speedLimit) {
    wchar_t text[32] = {};
    wchar_t* end = nullptr;
    GetWindowTextW(forwardEdit, text, 32);
    long value = wcstol(text, &end, 10);
    if (end == text || *end != L'\0' || value < 0 || value > 4) return false;
    forward = static_cast<int>(value);
    GetWindowTextW(backwardEdit, text, 32);
    value = wcstol(text, &end, 10);
    if (end == text || *end != L'\0' || value < 0 || value > 4) return false;
    backward = static_cast<int>(value);
    if (forward == 0 && backward == 0) return false;
    GetWindowTextW(speedLimitEdit, text, 32);
    for (wchar_t& ch : text) if (ch == L',') ch = L'.';
    const double kmh = wcstod(text, &end);
    if (end == text || *end != L'\0' || !std::isfinite(kmh) ||
        kmh < 10 || kmh > 180) return false;
    speedLimit = kmh / 3.6;
    return true;
}

void refreshSelection() {
    if (auto* point = spawnById(selectedSpawn)) {
        wchar_t value[32];
        swprintf(value, 32, L"%.2f", point->carsPerSecond());
        SetWindowTextW(rateEdit, value);
        swprintf(value, 32, L"%.1f", point->minInitialSpeed() * 3.6);
        SetWindowTextW(minInitialSpeedEdit, value);
        swprintf(value, 32, L"%.1f", point->maxInitialSpeed() * 3.6);
        SetWindowTextW(maxInitialSpeedEdit, value);
        EnableWindow(rateEdit, TRUE);
        EnableWindow(minInitialSpeedEdit, TRUE);
        EnableWindow(maxInitialSpeedEdit, TRUE);
        EnableWindow(GetDlgItem(mainWindow, idApply), TRUE);
    } else {
        SetWindowTextW(rateEdit, L"");
        SetWindowTextW(minInitialSpeedEdit, L"");
        SetWindowTextW(maxInitialSpeedEdit, L"");
        EnableWindow(rateEdit, FALSE);
        EnableWindow(minInitialSpeedEdit, FALSE);
        EnableWindow(maxInitialSpeedEdit, FALSE);
        EnableWindow(GetDlgItem(mainWindow, idApply), FALSE);
    }
    EnableWindow(GetDlgItem(mainWindow, idDelete),
                 selectedRoad >= 0 || selectedSpawn >= 0);
    InvalidateRect(mainWindow, nullptr, FALSE);
}

void setTool(Tool next) {
    tool = next;
    CheckDlgButton(mainWindow, idSelect, next == Tool::select ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(mainWindow, idRoad, next == Tool::road ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(mainWindow, idSpawn, next == Tool::spawn ? BST_CHECKED : BST_UNCHECKED);
    InvalidateRect(mainWindow, nullptr, FALSE);
}

void setRunning(bool value) {
    running = value;
    elapsed = 0;
    lastFrame = GetTickCount64();
    SetWindowTextW(runButton, running ? L"Пауза" : L"Старт");
    InvalidateRect(mainWindow, nullptr, FALSE);
}

bool readSpeedKmh(HWND edit, double& value) {
    wchar_t buffer[64] = {};
    GetWindowTextW(edit, buffer, 64);
    for (wchar_t& ch : buffer) if (ch == L',') ch = L'.';
    wchar_t* end = nullptr;
    value = wcstod(buffer, &end);
    return end != buffer && *end == L'\0' && std::isfinite(value);
}

void applySpawnSettings() {
    SpawnPoint* point = spawnById(selectedSpawn);
    if (!point) return;
    wchar_t buffer[64] = {};
    GetWindowTextW(rateEdit, buffer, 64);
    for (wchar_t& ch : buffer) if (ch == L',') ch = L'.';
    wchar_t* end = nullptr;
    double value = wcstod(buffer, &end);
    double minKmh = 0, maxKmh = 0;
    const double allowedKmh = std::min(13.9, point->road().speedLimit()) * 3.6;
    if (end == buffer || *end != L'\0' || !std::isfinite(value) ||
        value < 0 || value > 10 ||
        !readSpeedKmh(minInitialSpeedEdit, minKmh) ||
        !readSpeedKmh(maxInitialSpeedEdit, maxKmh) ||
        minKmh < 0 || maxKmh < minKmh || maxKmh > allowedKmh + 1e-8) {
        notice = L"Проверьте частоту и диапазон скорости (до лимита дороги и 50 км/ч).";
        MessageBeep(MB_ICONWARNING);
    } else {
        point->setCarsPerSecond(value);
        point->setInitialSpeedRange(minKmh / 3.6, maxKmh / 3.6);
        notice = L"Параметры точки входа изменены.";
    }
    refreshSelection();
}

void selectAt(Point p) {
    selectedRoad = selectedSpawn = -1;
    double best = 17;
    for (const auto& point : simulation.spawnPoints()) {
        if (simulation.hasIncomingConnection(point->road(), point->direction())) continue;
        double d = distance(p, point->road().lanePointAt(
            0, point->direction(), point->lane()));
        if (d < best) { best = d; selectedSpawn = point->id(); }
    }
    if (selectedSpawn < 0) {
        best = 12;
        for (const auto& road : simulation.roads()) {
            double d = distanceToRoad(p, *road);
            if (d < best) { best = d; selectedRoad = road->id(); }
        }
    }
    refreshSelection();
}

void createSpawn(Point p) {
    Road* nearest = nullptr;
    Direction nearestDirection = Direction::forward;
    int nearestLane = 0;
    double best = 14;
    for (const auto& road : simulation.roads()) {
        for (Direction direction : {Direction::forward, Direction::backward}) {
            if (simulation.hasIncomingConnection(*road, direction)) continue;
            for (int lane = 0; lane < road->laneCount(direction); ++lane) {
                double d = distance(p, road->lanePointAt(0, direction, lane));
                if (d < best) {
                    best = d;
                    nearest = road.get();
                    nearestDirection = direction;
                    nearestLane = lane;
                }
            }
        }
    }
    if (!nearest) {
        notice = L"Нажмите на зелёный вход нужной полосы.";
    } else {
        bool exists = false;
        for (const auto& point : simulation.spawnPoints())
            if (&point->road() == nearest &&
                point->direction() == nearestDirection &&
                point->lane() == nearestLane) exists = true;
        if (exists) {
            notice = L"На этой полосе уже есть точка спавна.";
        } else {
            selectedSpawn = simulation.addSpawnPoint(
                *nearest, nearestDirection, nearestLane).id();
            selectedRoad = -1;
            notice = L"Точка спавна добавлена. Задайте машин/с слева.";
            refreshSelection();
        }
    }
    InvalidateRect(mainWindow, nullptr, FALSE);
}

void deleteSelected() {
    if (selectedSpawn >= 0) simulation.removeSpawnPoint(selectedSpawn);
    else if (selectedRoad >= 0) simulation.removeRoad(selectedRoad);
    selectedSpawn = selectedRoad = -1;
    notice = L"Объект удалён.";
    refreshSelection();
}

HWND addControl(const wchar_t* kind, const wchar_t* text, DWORD style,
                int id, int x, int y, int w, int h) {
    HWND control = CreateWindowExW(0, kind, text, WS_CHILD | WS_VISIBLE | style,
                                   x, y, w, h, mainWindow,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(uiFont), TRUE);
    return control;
}

void drawText(HDC dc, const std::wstring& value, int x, int y, int w, int h,
              COLORREF color, UINT format = DT_LEFT | DT_WORDBREAK) {
    RECT r{x, y, x + w, y + h};
    SetTextColor(dc, color);
    SetBkMode(dc, TRANSPARENT);
    DrawTextW(dc, value.c_str(), -1, &r, format);
}

void line(HDC dc, int x1, int y1, int x2, int y2) {
    MoveToEx(dc, x1, y1, nullptr);
    LineTo(dc, x2, y2);
}

void drawRoad(HDC dc, const Road& road, bool selected) {
    Point a = road.start(), b = road.end();
    int ax = sidebar + static_cast<int>(std::lround(a.x));
    int ay = static_cast<int>(std::lround(a.y));
    int bx = sidebar + static_cast<int>(std::lround(b.x));
    int by = static_cast<int>(std::lround(b.y));
    for (Direction direction : {Direction::forward, Direction::backward}) {
        for (int lane = 0; lane < road.laneCount(direction); ++lane) {
            Point start = road.lanePointAt(0, direction, lane);
            Point end = road.lanePointAt(road.length(), direction, lane);
            HPEN surface = CreatePen(PS_SOLID, static_cast<int>(laneWidthPixels),
                                     selected ? RGB(58, 107, 159) : RGB(51, 62, 75));
            HPEN oldPen = static_cast<HPEN>(SelectObject(dc, surface));
            line(dc, sidebar + static_cast<int>(std::lround(start.x)),
                 static_cast<int>(std::lround(start.y)),
                 sidebar + static_cast<int>(std::lround(end.x)),
                 static_cast<int>(std::lround(end.y)));
            SelectObject(dc, oldPen);
            DeleteObject(surface);
        }
    }
    for (Direction direction : {Direction::forward, Direction::backward}) {
        for (int lane = 0; lane < road.laneCount(direction); ++lane) {
            Point start = road.lanePointAt(0, direction, lane);
            Point end = road.lanePointAt(road.length(), direction, lane);
            int x1 = sidebar + static_cast<int>(std::lround(start.x));
            int y1 = static_cast<int>(std::lround(start.y));
            int x2 = sidebar + static_cast<int>(std::lround(end.x));
            int y2 = static_cast<int>(std::lround(end.y));
            HPEN edge = CreatePen(PS_SOLID, 1, RGB(122, 133, 143));
            HPEN oldPen = static_cast<HPEN>(SelectObject(dc, edge));
            Point divider = road.lanePointAt(0, direction, lane);
            double shift = direction == Direction::forward ? laneWidthPixels / 2
                                                            : -laneWidthPixels / 2;
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double pixels = std::hypot(dx, dy);
            divider.x += -dy / pixels * shift;
            divider.y += dx / pixels * shift;
            line(dc, sidebar + static_cast<int>(std::lround(divider.x)),
                 static_cast<int>(std::lround(divider.y)),
                 sidebar + static_cast<int>(std::lround(divider.x + x2 - x1)),
                 static_cast<int>(std::lround(divider.y + y2 - y1)));
            SelectObject(dc, oldPen);
            DeleteObject(edge);

            Point arrowPoint = road.lanePointAt(road.length() * 0.63,
                                                direction, lane);
            const double vx = (end.x - start.x) / pixels;
            const double vy = (end.y - start.y) / pixels;
            const double cx = sidebar + arrowPoint.x, cy = arrowPoint.y;
            POINT arrow[3] = {
                {static_cast<LONG>(cx + vx * 6), static_cast<LONG>(cy + vy * 6)},
                {static_cast<LONG>(cx - vx * 4 - vy * 3), static_cast<LONG>(cy - vy * 4 + vx * 3)},
                {static_cast<LONG>(cx - vx * 4 + vy * 3), static_cast<LONG>(cy - vy * 4 - vx * 3)}
            };
            HBRUSH brush = CreateSolidBrush(RGB(235, 241, 245));
            HBRUSH oldBrush = static_cast<HBRUSH>(SelectObject(dc, brush));
            Polygon(dc, arrow, 3);
            SelectObject(dc, oldBrush);
            DeleteObject(brush);
            if (!simulation.hasIncomingConnection(road, direction)) {
                brush = CreateSolidBrush(RGB(42, 180, 126));
                oldBrush = static_cast<HBRUSH>(SelectObject(dc, brush));
                Ellipse(dc, x1 - 5, y1 - 5, x1 + 6, y1 + 6);
                SelectObject(dc, oldBrush);
                DeleteObject(brush);
            }
            if (!simulation.isConnectedEnd(road,
                direction == Direction::backward)) {
                brush = CreateSolidBrush(RGB(205, 83, 84));
                oldBrush = static_cast<HBRUSH>(SelectObject(dc, brush));
                Ellipse(dc, x2 - 5, y2 - 5, x2 + 6, y2 + 6);
                SelectObject(dc, oldBrush);
                DeleteObject(brush);
            }
        }
    }
    HPEN centre = CreatePen(PS_SOLID, 2, RGB(232, 190, 67));
    HPEN oldPen = static_cast<HPEN>(SelectObject(dc, centre));
    line(dc, ax, ay, bx, by);
    SelectObject(dc, oldPen);
    DeleteObject(centre);
    wchar_t label[96];
    swprintf(label, 96, L"Дорога %d · %.0f м · %d+%d · %.0f км/ч",
             road.id(), road.length(), road.laneCount(Direction::forward),
             road.laneCount(Direction::backward), road.speedLimit() * 3.6);
    drawText(dc, label, (ax + bx) / 2 - 85, (ay + by) / 2 - 40, 230, 24,
             selected ? RGB(40, 98, 169) : RGB(90, 101, 112));
    if (selected) {
        HBRUSH handle = CreateSolidBrush(RGB(54, 133, 225));
        HBRUSH oldBrush = static_cast<HBRUSH>(SelectObject(dc, handle));
        Ellipse(dc, ax - 7, ay - 7, ax + 8, ay + 8);
        Ellipse(dc, bx - 7, by - 7, bx + 8, by + 8);
        SelectObject(dc, oldBrush);
        DeleteObject(handle);
    }
}

void drawScene(HDC dc, RECT client) {
    HBRUSH panel = CreateSolidBrush(RGB(242, 246, 250));
    RECT side{0, 0, sidebar, client.bottom};
    FillRect(dc, &side, panel);
    DeleteObject(panel);
    HBRUSH canvas = CreateSolidBrush(RGB(249, 251, 253));
    RECT scene{sidebar, 0, client.right, client.bottom};
    FillRect(dc, &scene, canvas);
    DeleteObject(canvas);
    HPEN grid = CreatePen(PS_SOLID, 1, RGB(231, 236, 241));
    HPEN oldPen = static_cast<HPEN>(SelectObject(dc, grid));
    for (int x = sidebar; x < client.right; x += 40) line(dc, x, 0, x, client.bottom);
    for (int y = 0; y < client.bottom; y += 40) line(dc, sidebar, y, client.right, y);
    SelectObject(dc, oldPen);
    DeleteObject(grid);

    HFONT oldFont = static_cast<HFONT>(SelectObject(dc, uiFont));
    drawText(dc, L"Схема движения", margin, 17, controlWidth, 34, RGB(26, 42, 59));
    drawText(dc, L"ИНСТРУМЕНТЫ", margin, 60, controlWidth, 22, RGB(83, 103, 121));
    drawText(dc, L"НОВАЯ ДОРОГА", margin, 194, controlWidth, 22, RGB(83, 103, 121));
    drawText(dc, L"Полос →", margin, 218, 80, 22, RGB(50, 65, 80));
    drawText(dc, L"Полос ←", 109, 218, 80, 22, RGB(50, 65, 80));
    drawText(dc, L"км/ч", 207, 218, 78, 22, RGB(50, 65, 80));
    drawText(dc, L"СИМУЛЯЦИЯ", margin, 288, controlWidth, 22, RGB(83, 103, 121));
    drawText(dc, L"ГЕНЕРАТОР ВЫБРАННОЙ ПОЛОСЫ", margin, 455, controlWidth, 22, RGB(83, 103, 121));
    drawText(dc, L"Машин в секунду (0–10):", margin, 478, controlWidth, 22, RGB(50, 65, 80));
    drawText(dc, L"Начальная скорость, км/ч", margin, 540, controlWidth, 22, RGB(50, 65, 80));
    drawText(dc, L"От", margin, 563, 90, 22, RGB(50, 65, 80));
    drawText(dc, L"До", 160, 563, 90, 22, RGB(50, 65, 80));
    drawText(dc, L"ВЫБРАННЫЙ ОБЪЕКТ", margin, 630, controlWidth, 22, RGB(83, 103, 121));

    wchar_t summary[256];
    swprintf(summary, 256, L"Время: %.1f с\nНа дороге: %zu\nСоздано: %zu  ·  Выехало: %zu",
             simulation.time(), simulation.cars().size(),
             simulation.spawnedCars(), simulation.exitedCars());
    drawText(dc, summary, margin, 383, controlWidth, 63, RGB(39, 58, 73));

    std::wstring selection = L"Ничего не выбрано";
    if (auto* point = spawnById(selectedSpawn)) {
        wchar_t info[128];
        swprintf(info, 128, L"Точка %d · дорога %d · %ls %d\n%.2f машин/с · %.0f–%.0f км/ч",
                 point->id(), point->road().id(),
                 point->direction() == Direction::forward ? L"→" : L"←",
                 point->lane() + 1, point->carsPerSecond(),
                 point->minInitialSpeed() * 3.6,
                 point->maxInitialSpeed() * 3.6);
        selection = info;
    } else if (auto* road = roadById(selectedRoad)) {
        wchar_t info[128];
        swprintf(info, 128, L"Дорога %d · %.1f м\nПолос: %d → / %d ← · %.0f км/ч",
                 road->id(), road->length(), road->laneCount(Direction::forward),
                 road->laneCount(Direction::backward), road->speedLimit() * 3.6);
        selection = info;
    }
    drawText(dc, selection, margin, 653, controlWidth, 58, RGB(39, 58, 73));
    drawText(dc, L"Нарисуйте дорогу. В режиме «Выбрать» тяните дорогу или её синие концы. "
                 L"Близкие концы соединяются. Зелёные точки — входы, красные — выходы.",
             margin, 804, controlWidth, 78, RGB(94, 108, 121));
    drawText(dc, notice, margin, 902, controlWidth, 70, RGB(53, 111, 154));

    int saved = SaveDC(dc);
    IntersectClipRect(dc, sidebar, 0, client.right, client.bottom);
    for (const auto& road : simulation.roads())
        drawRoad(dc, *road, road->id() == selectedRoad);
    for (const auto& point : simulation.spawnPoints()) {
        if (simulation.hasIncomingConnection(point->road(), point->direction())) continue;
        Point p = point->road().lanePointAt(0, point->direction(), point->lane());
        int x = sidebar + static_cast<int>(std::lround(p.x));
        int y = static_cast<int>(std::lround(p.y));
        HBRUSH brush = CreateSolidBrush(point->id() == selectedSpawn ?
                                        RGB(31, 107, 206) : RGB(30, 145, 204));
        HBRUSH old = static_cast<HBRUSH>(SelectObject(dc, brush));
        Ellipse(dc, x - 8, y - 8, x + 9, y + 9);
        SelectObject(dc, old);
        DeleteObject(brush);
        drawText(dc, L"S", x - 5, y - 9, 12, 20, RGB(255, 255, 255));
    }
    for (const auto& car : simulation.cars()) {
        const Lane& lane = simulation.laneById(car->laneId());
        const Road& road = simulation.roadForLane(car->laneId());
        Point p = road.lanePointAt(car->position() + car->length() / 2,
                                   lane.direction(), lane.index());
        const double a = std::atan2(road.end().y - road.start().y,
                                    road.end().x - road.start().x) +
                         (lane.direction() == Direction::backward ? 3.141592653589793 : 0);
        const double ux = std::cos(a), uy = std::sin(a);
        const double halfLength = car->length() / metresPerPixel / 2;
        const double halfWidth = 5;
        const double cx = sidebar + p.x, cy = p.y;
        POINT body[4] = {
            {static_cast<LONG>(cx + ux * halfLength - uy * halfWidth), static_cast<LONG>(cy + uy * halfLength + ux * halfWidth)},
            {static_cast<LONG>(cx + ux * halfLength + uy * halfWidth), static_cast<LONG>(cy + uy * halfLength - ux * halfWidth)},
            {static_cast<LONG>(cx - ux * halfLength + uy * halfWidth), static_cast<LONG>(cy - uy * halfLength - ux * halfWidth)},
            {static_cast<LONG>(cx - ux * halfLength - uy * halfWidth), static_cast<LONG>(cy - uy * halfLength + ux * halfWidth)}
        };
        HBRUSH brush = CreateSolidBrush(RGB(255, 176, 62));
        HBRUSH old = static_cast<HBRUSH>(SelectObject(dc, brush));
        Polygon(dc, body, 4);
        SelectObject(dc, old);
        DeleteObject(brush);
    }
    if (dragging && dragMode == DragMode::newRoad) {
        HPEN preview = CreatePen(PS_DASH, 3, RGB(59, 132, 214));
        oldPen = static_cast<HPEN>(SelectObject(dc, preview));
        line(dc, sidebar + static_cast<int>(dragStart.x), static_cast<int>(dragStart.y),
             sidebar + static_cast<int>(dragEnd.x), static_cast<int>(dragEnd.y));
        SelectObject(dc, oldPen);
        DeleteObject(preview);
    }
    RestoreDC(dc, saved);
    SelectObject(dc, oldFont);
}

LRESULT CALLBACK windowProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) {
    switch (msg) {
    case WM_GETMINMAXINFO: {
        auto* limits = reinterpret_cast<MINMAXINFO*>(lparam);
        limits->ptMinTrackSize = {1050, 1040};
        return 0;
    }
    case WM_CREATE: {
        mainWindow = hwnd;
        uiFont = CreateFontW(-17, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                             DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                             CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
        addControl(L"BUTTON", L"Выбрать", BS_AUTORADIOBUTTON | WS_GROUP,
                   idSelect, margin, 83, controlWidth, 30);
        addControl(L"BUTTON", L"Дорога", BS_AUTORADIOBUTTON,
                   idRoad, margin, 117, controlWidth, 30);
        addControl(L"BUTTON", L"Точка спавна", BS_AUTORADIOBUTTON,
                   idSpawn, margin, 151, controlWidth, 30);
        forwardEdit = addControl(L"EDIT", L"2", WS_BORDER | ES_NUMBER,
                                 idForwardLanes, margin, 241, 70, 30);
        backwardEdit = addControl(L"EDIT", L"2", WS_BORDER | ES_NUMBER,
                                  idBackwardLanes, 109, 241, 70, 30);
        speedLimitEdit = addControl(L"EDIT", L"50", WS_BORDER | ES_AUTOHSCROLL,
                                    idSpeedLimit, 207, 241, 70, 30);
        runButton = addControl(L"BUTTON", L"Старт", BS_PUSHBUTTON,
                               idRun, margin, 310, 130, 34);
        addControl(L"BUTTON", L"Шаг +0,05 с", BS_PUSHBUTTON,
                   idStep, 162, 310, 130, 34);
        addControl(L"BUTTON", L"Сброс времени", BS_PUSHBUTTON,
                   idReset, margin, 348, controlWidth, 28);
        rateEdit = addControl(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                              idRate, margin, 503, 120, 30);
        addControl(L"BUTTON", L"Применить", BS_PUSHBUTTON,
                   idApply, 153, 502, 139, 31);
        minInitialSpeedEdit = addControl(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                                         idMinInitialSpeed, margin, 583, 115, 30);
        maxInitialSpeedEdit = addControl(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                                         idMaxInitialSpeed, 160, 583, 115, 30);
        addControl(L"BUTTON", L"Удалить выбранное", BS_PUSHBUTTON,
                   idDelete, margin, 716, controlWidth, 30);
        addControl(L"BUTTON", L"Очистить схему", BS_PUSHBUTTON,
                   idClear, margin, 758, controlWidth, 30);
        setTool(Tool::road);
        refreshSelection();
        SetTimer(hwnd, 1, 16, nullptr);
        lastFrame = GetTickCount64();
        return 0;
    }
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case idSelect: setTool(Tool::select); break;
        case idRoad: setTool(Tool::road); break;
        case idSpawn: setTool(Tool::spawn); break;
        case idRun: setRunning(!running); break;
        case idStep: if (!running) { simulation.tick(); InvalidateRect(hwnd, nullptr, FALSE); } break;
        case idReset: setRunning(false); simulation.reset(); InvalidateRect(hwnd, nullptr, FALSE); break;
        case idDelete: setRunning(false); deleteSelected(); break;
        case idClear:
            setRunning(false);
            simulation.clear();
            selectedRoad = selectedSpawn = -1;
            notice = L"Схема очищена.";
            refreshSelection();
            break;
        case idApply: applySpawnSettings(); break;
        }
        return 0;
    case WM_LBUTTONDOWN: {
        POINT screen{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
        if (!onCanvas(screen)) return 0;
        Point p = mousePoint(lparam);
        if (tool == Tool::road) {
            setRunning(false);
            dragging = true;
            dragMode = DragMode::newRoad;
            dragStart = dragEnd = snappedEndpoint(p);
            SetCapture(hwnd);
        } else if (tool == Tool::spawn) {
            setRunning(false);
            createSpawn(p);
        } else {
            bool spawnHit = false;
            for (const auto& point : simulation.spawnPoints()) {
                if (simulation.hasIncomingConnection(point->road(), point->direction()))
                    continue;
                if (distance(p, point->road().lanePointAt(
                    0, point->direction(), point->lane())) < 8) {
                    spawnHit = true;
                    break;
                }
            }
            if (Road* selected = roadById(selectedRoad))
                if (distance(p, selected->start()) < 11 ||
                    distance(p, selected->end()) < 11)
                    spawnHit = false;
            bool atStart = false;
            Road* handle = spawnHit ? nullptr : roadHandleAt(p, atStart);
            if (handle) {
                selectedRoad = handle->id();
                selectedSpawn = -1;
                dragMode = atStart ? DragMode::start : DragMode::end;
                refreshSelection();
            } else {
                selectAt(p);
                if (selectedRoad >= 0) dragMode = DragMode::moveRoad;
            }
            if (dragMode != DragMode::none) {
                setRunning(false);
                dragging = true;
                editStarted = false;
                pointerStart = p;
                Road* road = roadById(selectedRoad);
                originalStart = road->start();
                originalEnd = road->end();
                SetCapture(hwnd);
            }
        }
        return 0;
    }
    case WM_MOUSEMOVE:
        if (dragging) {
            if (dragMode == DragMode::newRoad) {
                dragEnd = snappedEndpoint(mousePoint(lparam));
                InvalidateRect(hwnd, nullptr, FALSE);
            } else updateRoadDrag(mousePoint(lparam));
        }
        return 0;
    case WM_LBUTTONUP:
        if (dragging) {
            ReleaseCapture();
            dragging = false;
            if (dragMode == DragMode::newRoad) {
                dragEnd = snappedEndpoint(mousePoint(lparam));
                if (distance(dragStart, dragEnd) >= 35) {
                    int forward, backward;
                    double limit;
                    if (!roadParameters(forward, backward, limit)) {
                        notice = L"Укажите 0–4 полосы (хотя бы одну всего) и 10–180 км/ч.";
                    } else {
                        try {
                            selectedRoad = simulation.addRoad(
                                dragStart, dragEnd, forward, backward, limit).id();
                            selectedSpawn = -1;
                            notice = L"Дорога добавлена. Совпавшие концы соединены.";
                            refreshSelection();
                        } catch (...) { notice = L"Проверьте длину и параметры дороги."; }
                    }
                } else notice = L"Проведите дорогу длиной хотя бы 35 пикселей.";
            } else {
                updateRoadDrag(mousePoint(lparam));
                if (editStarted) {
                    notice = L"Дорога изменена. Машины и время сброшены.";
                    refreshSelection();
                }
            }
            dragMode = DragMode::none;
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_TIMER:
        if (running) {
            ULONGLONG now = GetTickCount64();
            elapsed += std::min(0.25, (now - lastFrame) / 1000.0);
            lastFrame = now;
            while (elapsed >= tickSeconds) {
                simulation.tick();
                elapsed -= tickSeconds;
            }
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(hwnd, &ps);
        RECT client;
        GetClientRect(hwnd, &client);
        HDC memory = CreateCompatibleDC(dc);
        HBITMAP bitmap = CreateCompatibleBitmap(dc, std::max(1L, client.right),
                                                std::max(1L, client.bottom));
        HBITMAP old = static_cast<HBITMAP>(SelectObject(memory, bitmap));
        drawScene(memory, client);
        BitBlt(dc, 0, 0, client.right, client.bottom, memory, 0, 0, SRCCOPY);
        SelectObject(memory, old);
        DeleteObject(bitmap);
        DeleteDC(memory);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_ERASEBKGND: return 1;
    case WM_DESTROY:
        KillTimer(hwnd, 1);
        DeleteObject(uiFont);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wparam, lparam);
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show) {
    WNDCLASSEXW cls{};
    cls.cbSize = sizeof(cls);
    cls.hInstance = instance;
    cls.lpszClassName = L"TrafficSimulatorWindow";
    cls.lpfnWndProc = windowProc;
    cls.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    cls.hIcon = LoadIconW(nullptr, IDI_APPLICATION);
    RegisterClassExW(&cls);
    HWND hwnd = CreateWindowExW(0, cls.lpszClassName, L"Симулятор дорожного движения",
                                WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                                1280, 1080, nullptr, nullptr, instance, nullptr);
    if (!hwnd) return 1;
    ShowWindow(hwnd, show);
    UpdateWindow(hwnd);
    MSG message;
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}
