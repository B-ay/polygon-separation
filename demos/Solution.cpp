#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <functional>

struct Vector2D;
struct Polygon;

#define ENABLE_TIMER 1

using namespace std;
const double EPS = 1e-6;

struct Vector2D {
    double x, y;

    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
    Vector2D operator+(const Vector2D& other) const { return Vector2D(x + other.x, y + other.y); }
    Vector2D operator*(double scalar) const { return Vector2D(x * scalar, y * scalar); }
    double Dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    double Cross(const Vector2D& other) const { return x * other.y - y * other.x; }
    double Length() const { return std::sqrt(x * x + y * y); }
    Vector2D Normalize() const
    {
        double len = Length();
        if (len == 0) {
            return *this;
        }
        return Vector2D(x / len, y / len);
    }
    Vector2D Perp() const { return Vector2D(-y, x); }
};

struct Projection {
    double min, max;
    
    // 应用平移修正
    void ApplyTranslation(double translation) {
        min += translation;
        max += translation;
    }
};

struct ConvexPolygon {
    std::vector<Vector2D> vertices;
    std::vector<Vector2D> axes;              // 预计算的边法线
    std::vector<Projection> selfProjections; // 多边形自身在所有法线上的投影区间
    std::vector<Projection> otherProjections; // 多边形在另一个多边形法线上的投影区间（用于交叉投影）
    Vector2D center;                         // 预计算的中心点

    ConvexPolygon() = default;
    ConvexPolygon(const std::vector<Vector2D>& vts) : vertices(vts) {}
    
    // 计算多边形的中心点
    Vector2D GetCenter() const
    {
        Vector2D center(0, 0);
        if (vertices.empty()) {
            return center;
        }
        for (const auto& v : vertices) {
            center = center + v;
        }
        return center * (1.0 / vertices.size());
    }

    // 将多边形整体移动vec向量
    void MoveByVec(const Vector2D& vec) {
        for (auto& v : vertices) {
            v = v + vec;
        }
        // 移动后需要更新中心点
        center = center + vec;
    }
    
    // 预计算所有边的法线
    void PrecomputeAxes() {
        axes.clear();
        size_t n = vertices.size();
        if (n == 0) return;
        
        axes.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            Vector2D p1 = vertices[i];
            Vector2D p2 = vertices[(i + 1) % n];
            Vector2D edge = p2 - p1;
            
            // 计算法线并归一化
            Vector2D axis = edge.Perp().Normalize();
            axes.push_back(axis);
        }
    }
    
    // 预计算多边形自身在所有法线上的投影区间
    void PrecomputeSelfProjections() {
        size_t axisCount = axes.size();
        if (axisCount == 0) return;
        
        selfProjections.resize(axisCount);
        
        for (size_t i = 0; i < axisCount; ++i) {
            const Vector2D& axis = axes[i];
            
            // 计算在当前法线上的投影区间
            double minProj = vertices[0].Dot(axis);
            double maxProj = minProj;
            
            for (size_t j = 1; j < vertices.size(); ++j) {
                double proj = vertices[j].Dot(axis);
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
            }
            
            selfProjections[i] = {minProj, maxProj};
        }
    }

    // 预计算本多边形在另一个多边形法线上的投影区间
    void PrecomputeOtherProjections(const std::vector<Vector2D>& otherAxes) {
        size_t axisCount = otherAxes.size();
        if (axisCount == 0) return;
        
        otherProjections.resize(axisCount);
        
        for (size_t i = 0; i < axisCount; ++i) {
            const Vector2D& axis = otherAxes[i];
            
            // 计算在当前法线上的投影区间
            double minProj = vertices[0].Dot(axis);
            double maxProj = minProj;
            
            for (size_t j = 1; j < vertices.size(); ++j) {
                double proj = vertices[j].Dot(axis);
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
            }
            
            otherProjections[i] = {minProj, maxProj};
        }
    }
    
    // 预计算中心点
    void PrecomputeCenter() {
        center = GetCenter();
    }

    // 快速获取自身平移后的投影区间
    Projection GetSelfProjectionWithTranslation(const Vector2D& translation, size_t axisIndex) const {
        double translationProj = translation.Dot(axes[axisIndex]);
        Projection result = selfProjections[axisIndex];
        result.ApplyTranslation(translationProj);
        return result;
    }
    
    // 快速获取自身平移后在其他多边形法线上的投影区间
    Projection GetOtherProjectionWithTranslation(const Vector2D& translation, 
                                                  const Vector2D& axis, 
                                                  size_t projectionIndex) const {
        double translationProj = translation.Dot(axis);
        Projection result = otherProjections[projectionIndex];
        result.ApplyTranslation(translationProj);
        return result;
    }
};

// 凹多边形类，由多个凸多边形组成
struct ConcavePolygon {
    std::vector<ConvexPolygon> convexParts;  // 分解后的凸多边形部分
    std::vector<Vector2D> originalVertices;  // 原始顶点
    Vector2D center;                         // 中心点
    
    ConcavePolygon() = default;
    
    // 判断点是否在多边形内部
    // bool IsPointInPolygon(const Vector2D& point) const {
    //     bool inside = false;
    //     int n = originalVertices.size();
    //     for (int i = 0, j = n - 1; i < n; j = i++) {
    //         if (((originalVertices[i].y > point.y) != (originalVertices[j].y > point.y)) &&
    //             (point.x < (originalVertices[j].x - originalVertices[i].x) * (point.y - originalVertices[i].y) / 
    //                      (originalVertices[j].y - originalVertices[i].y) + originalVertices[i].x)) {
    //             inside = !inside;
    //         }
    //     }
    //     return inside;
    // }
    
    // 判断点是否在凸多边形内部
    // bool IsPointInConvex(const Vector2D& point, const ConvexPolygon& convex) const {
    //     int n = convex.vertices.size();
    //     for (int i = 0; i < n; i++) {
    //         Vector2D edge = convex.vertices[(i + 1) % n] - convex.vertices[i];
    //         Vector2D toPoint = point - convex.vertices[i];
    //         if (edge.Cross(toPoint) < -EPS) {
    //             return false;
    //         }
    //     }
    //     return true;
    // }
    
    // 耳切法三角剖分
    std::vector<ConvexPolygon> Triangulate() {
        std::vector<ConvexPolygon> triangles;
        std::vector<Vector2D> pts = originalVertices;
        int n = pts.size();

        if (n < 3) return triangles;

        while (pts.size() > 3) {
            bool earFound = false;

            for (int i = 0; i < pts.size(); ++i) {
                int prevIdx = (i - 1 + pts.size()) % pts.size();
                int nextIdx = (i + 1) % pts.size();

                Vector2D& A = pts[prevIdx];
                Vector2D& B = pts[i];
                Vector2D& C = pts[nextIdx];

                // 判断是否为凸顶点（逆时针多边形）
                Vector2D AB = B - A;
                Vector2D BC = C - B;
                if (AB.Cross(BC) < -EPS)
                    continue;

                // 检查是否有点在三角形内
                bool hasPointInside = false;
                for (int j = 0; j < pts.size(); ++j) {
                    if (j == prevIdx || j == i || j == nextIdx)
                        continue;
                    if (IsPointInTriangle(pts[j], A, B, C)) {
                        hasPointInside = true;
                        break;
                    }
                }

                if (!hasPointInside) {
                    triangles.emplace_back(std::vector<Vector2D>{A, B, C});
                    pts.erase(pts.begin() + i);
                    earFound = true;
                    break;
                }
            }

            if (!earFound) break;
        }

        if (pts.size() == 3) {
            triangles.emplace_back(pts);
        }

        return triangles;
    }
    
    bool IsPointInTriangle(const Vector2D& p, const Vector2D& a, const Vector2D& b, const Vector2D& c) const {
        Vector2D v0 = c - a;
        Vector2D v1 = b - a;
        Vector2D v2 = p - a;

        double dot00 = v0.Dot(v0);
        double dot01 = v0.Dot(v1);
        double dot02 = v0.Dot(v2);
        double dot11 = v1.Dot(v1);
        double dot12 = v1.Dot(v2);

        double denom = dot00 * dot11 - dot01 * dot01;
        if (fabs(denom) < EPS) return false;

        double invDenom = 1.0 / denom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= -EPS) && (v >= -EPS) && (u + v <= 1.0 + EPS);
    }
    
    // 预计算所有凸部分的投影数据
    void Precompute() {
        // 使用三角剖分将凹多边形分解为凸多边形
        convexParts = Triangulate();
        
        // 预计算每个凸部分的投影数据
        for (auto& part : convexParts) {
            part.PrecomputeAxes();
            part.PrecomputeSelfProjections();
            part.PrecomputeCenter();
        }
        
        // 计算中心点
        center = Vector2D(0, 0);
        for (const auto& v : originalVertices) {
            center = center + v;
        }
        center = center * (1.0 / originalVertices.size());
    }
    
    // 移动多边形
    void MoveByVec(const Vector2D& vec) {
        for (auto& v : originalVertices) {
            v = v + vec;
        }
        for (auto& part : convexParts) {
            part.MoveByVec(vec);
        }
        center = center + vec;
    }
    
    // 预计算交叉投影
    void PrecomputeCrossProjections(const ConcavePolygon& other) {
        // 为每个凸部分预计算与另一个多边形所有凸部分的交叉投影
        for (auto& part : convexParts) {
            // 收集另一个多边形所有凸部分的轴线
            std::vector<Vector2D> allAxes;
            for (const auto& otherPart : other.convexParts) {
                for (const auto& axis : otherPart.axes) {
                    allAxes.push_back(axis);
                }
            }
            // 去重（可选，为简单起见先不做去重）
            part.PrecomputeOtherProjections(allAxes);
        }
        
        // for (auto& otherPart : other.convexParts) {
        //     std::vector<Vector2D> allAxes;
        //     for (const auto& part : convexParts) {
        //         for (const auto& axis : part.axes) {
        //             allAxes.push_back(axis);
        //         }
        //     }
        //     otherPart.PrecomputeOtherProjections(allAxes);
        // }
    }
};

int n1 = 0, n2 = 0, m = 0;
ConcavePolygon polygon1;
ConcavePolygon polygon2;
vector<Vector2D> testCases;

// 通用的投影函数（用于计算非预计算轴上的投影）
Projection ProjectPolygon(const ConvexPolygon& poly, const Vector2D& axis)
{
    double minProj = poly.vertices[0].Dot(axis);
    double maxProj = minProj;

    for (size_t i = 1; i < poly.vertices.size(); ++i) {
        double proj = poly.vertices[i].Dot(axis);
        if (proj < minProj) {
            minProj = proj;
        }
        if (proj > maxProj) {
            maxProj = proj;
        }
    }
    return {minProj, maxProj};
}

// 检查两个凸多边形的最小平移向量
Vector2D ComputeMTV(const ConvexPolygon& A, const ConvexPolygon& B, const Vector2D& vec) {
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D smallestAxis;
    
    // 检查当前多边形的每条法线
    for (size_t j = 0; j < A.axes.size(); ++j) {
        const Vector2D& axis = A.axes[j];
        
        const Projection& projA = A.selfProjections[j];
        Projection projB = B.GetOtherProjectionWithTranslation(vec, axis, j);
        
        if (projA.max <= projB.min || projB.max <= projA.min) {
            return {0.0, 0.0};
        }
        
        double overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }
    
    // 检查另一个多边形的每条法线
    for (size_t j = 0; j < B.axes.size(); ++j) {
        const Vector2D& axis = B.axes[j];
        
        Projection projA = B.otherProjections[j];
        Projection projB = B.GetSelfProjectionWithTranslation(vec, j);
        
        if (projA.max <= projB.min || projB.max <= projA.min) {
            return {0.0, 0.0};
        }
        
        double overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // 计算方向并修正MTV
    Vector2D dir = (B.center + vec) - A.center;
    
    if (smallestAxis.Dot(dir) < 0.0) {
        smallestAxis = smallestAxis * -1.0;
    }

    // std::ofstream outfile("./log.txt", std::ios::app);
    // if (!outfile.is_open()) {
    //     std::cerr << "错误：无法创建文件 sample/timer_result.txt" << std::endl;
    //     return 1;
    // }
    // outfile << "MTV: (" << smallestAxis.x * minOverlap << ", " << smallestAxis.y * minOverlap << ")" << std::endl;
    return {smallestAxis * minOverlap};
}

// 检查两个凹多边形是否相交，并计算最小平移向量
Vector2D GenSolution(const Vector2D& vec)
{
    // 将多边形2平移
    // ConcavePolygon tempPoly2 = polygon2;
    // tempPoly2.MoveByVec(vec);
    
    double minOverlap = 0; //std::numeric_limits<double>::infinity();
    Vector2D bestMTV(0, 0);
    bool hasCollision = false;
    
    // 检查所有凸部分对
    for (const auto& part1 : polygon1.convexParts) {
        for (const auto& part2 : polygon2.convexParts) {
            // 快速包围盒检测（可选）
            
            // 获取两个凸多边形的MTV
            Vector2D mtv = ComputeMTV(part1, part2, vec);
            
            // 如果不为0，说明有碰撞
            if (mtv.x != 0 || mtv.y != 0) {
                hasCollision = true;
                double overlap = mtv.Length();
                if (overlap > minOverlap) {
                    minOverlap = overlap;
                    bestMTV = mtv;
                }
            }
        }
    }
    // std::ofstream outfile("./log.txt", std::ios::app);
    // if (!outfile.is_open()) {
    //     std::cerr << "错误：无法创建文件 sample/timer_result.txt" << std::endl;
    //     return 1;
    // }
    // outfile << "polygon1切割成 " << polygon1.convexParts.size() << " 个凸多边形，polygon2切割成 " << polygon2.convexParts.size() << " 个凸多边形" << std::endl;
    // 如果有碰撞，返回最小的MTV
    if (hasCollision) {
        // outfile << "最小平移向量: (" << bestMTV.x << ", " << bestMTV.y << ")" << std::endl;
        return bestMTV;
    }
    
    // 没有碰撞
    // outfile << "没有碰撞，返回 (0, 0)" << std::endl;
    return {0.0, 0.0};
}

// 选手在规定的时间内进行预处理，完成后返回OK
void PreProcess()
{
    // 存储原始顶点
    // polygon1.originalVertices = polygon1.convexParts.empty() ? 
    //     std::vector<Vector2D>() : polygon1.convexParts[0].vertices;
    // polygon2.originalVertices = polygon2.convexParts.empty() ? 
    //     std::vector<Vector2D>() : polygon2.convexParts[0].vertices;
    
    // 预计算凹多边形（包括三角剖分）
    polygon1.Precompute();
    polygon2.Precompute();
    
    // 预计算交叉投影
    polygon1.PrecomputeCrossProjections(polygon2);
    polygon2.PrecomputeCrossProjections(polygon1);
}

int main()
{
    // =============== 1. read polygons ===================
    cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        cerr << "Input data error: the number of vertices of both polygons should be greater than 2" << endl;
        return 1;
    }

    polygon1.originalVertices.resize(n1);
    for (int i = 0; i < n1; ++i) {
        cin >> polygon1.originalVertices[i].x >> polygon1.originalVertices[i].y;
    }

    polygon2.originalVertices.resize(n2);
    for (int i = 0; i < n2; ++i) {
        cin >> polygon2.originalVertices[i].x >> polygon2.originalVertices[i].y;
    }

    // wait for OK to ensure all polygon data is received
    string okResp;
    cin >> okResp;
    if (okResp != "OK") {
        cerr << "Input data error: waiting for OK after obtaining polygons but I get " << okResp << endl;
        return 0;
    }

    // ============== 2. preprocess ===================
    PreProcess();
    // send OK after finishing preprocessing
    cout << "OK" << endl;
    cout.flush();

    // ============== 3. read test data ===================
    cin >> m;
    testCases.resize(m);

    for (int i = 0; i < m; ++i) {
        cin >> testCases[i].x >> testCases[i].y;
    }

    // wait for OK to ensure all test cases are received
    cin >> okResp;
    if (okResp != "OK") {
        cerr << "Input data error: waiting for OK after that I have received all test points but I get " << okResp
             << endl;
        return 0;
    }

    #if ENABLE_TIMER
    // 打开输出文件
    std::ofstream outfile("./timer_result.txt");
    if (!outfile.is_open()) {
        std::cerr << "错误：无法创建文件 sample/timer_result.txt" << std::endl;
        return 1;
    }
    // 开始计时
    auto start = std::chrono::high_resolution_clock::now();
    #endif
    // ================ 4. solve and output results ===================
    // cout << m << endl;
    for (int i = 0; i < m; ++i) {
        const Vector2D& res = GenSolution(testCases[i]);
        cout << fixed << std::setprecision(5) << res.x << " " << res.y << endl;
        cout.flush();
    }

    // send OK after outputting all answer
    cout << "OK" << endl;
    cout.flush();
    #if ENABLE_TIMER
    // 结束计时
    auto end = std::chrono::high_resolution_clock::now();
    
    // 计算耗时
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // 输出结果
    outfile << "程序执行时间: " << duration.count() << " 微秒" << std::endl;
    outfile << "即: " << duration.count() / 1000.0 << " 毫秒" << std::endl;
    outfile << "即: " << duration.count() / 1000000.0 << " 秒" << std::endl;
    #endif
    return 0;
}