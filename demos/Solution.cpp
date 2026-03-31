#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <thread>
#include <mutex>
// #include <future>
#include <atomic>

struct Vector2D;
struct Polygon;

#define ENABLE_TIMER 1
#define NUM_THREADS 16  // 可以根据需要调整线程数

using namespace std;
const double EPS = 1e-6;

struct Vector2D {
    double x, y;

    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
    Vector2D operator+(const Vector2D& other) const { return Vector2D(x + other.x, y + other.y); }
    Vector2D operator*(double scalar) const { return Vector2D(x * scalar, y * scalar); }
    double Dot(const Vector2D& other) const { return x * other.x + y * other.y; }
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

struct Polygon {
    std::vector<Vector2D> vertices;
    std::vector<Vector2D> axes;              // 预计算的边法线
    std::vector<Projection> selfProjections; // 多边形在自身所有法线上的投影区间
    std::vector<Projection> otherProjections; // 多边形在另一个多边形法线上的投影区间（用于交叉投影）
    Vector2D center;                          // 预计算的中心点

    Polygon() = default;
    Polygon(std::initializer_list<Vector2D> vts) : vertices(vts) {}
    
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
        // 注意：selfProjections不需要更新，因为我们在使用时动态修正
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
    // 输入：另一个多边形的法线集合
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

int n1 = 0, n2 = 0, m = 0;
Polygon polygon1;
Polygon polygon2;
vector<Vector2D> testCases;

// 通用的投影函数（用于计算非预计算轴上的投影）
Projection ProjectPolygon(const Polygon& poly, const Vector2D& axis)
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

Vector2D GenSolution(const Vector2D& vec)
{
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D smallestAxis;
    
    // 多边形A固定不动，多边形B平移vec
    // 注意：polygon2的selfProjections是基于原始位置的，平移后需要修正
    
    // 检查多边形1的每条法线
    for (size_t j = 0; j < polygon1.axes.size(); ++j) {
        const Vector2D& axis = polygon1.axes[j];
        
        // 多边形A的投影（固定）
        const Projection& projA = polygon1.selfProjections[j];
        // 多边形B平移后的投影
        Projection projB = polygon2.GetOtherProjectionWithTranslation(vec, axis, j);
        
        // 分离轴检测
        if (projA.max <= projB.min || projB.max <= projA.min) {
            return {0.0, 0.0};
        }
        
        double overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }
    
    // 检查多边形2的每条法线
    for (size_t j = 0; j < polygon2.axes.size(); ++j) {
        const Vector2D& axis = polygon2.axes[j];
        
        // 多边形A：
        Projection projA = polygon1.otherProjections[j]; // 直接使用预计算的交叉投影
        // 多边形B平移后的投影（使用快速修正）
        Projection projB = polygon2.GetSelfProjectionWithTranslation(vec, j);
        
        // 分离轴检测
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
    Vector2D dir = (polygon2.center + vec) - polygon1.center;
    
    if (smallestAxis.Dot(dir) < 0.0) {
        smallestAxis = smallestAxis * -1.0;
    }

    return {smallestAxis * minOverlap};
}

// 线程工作函数：处理一段连续范围的测试用例
void ProcessTestCaseRange(int startIdx, int endIdx, 
                          std::vector<Vector2D>& results,
                          std::mutex& resultsMutex) {
    std::vector<Vector2D> localResults;
    localResults.reserve(endIdx - startIdx);
    
    for (int i = startIdx; i < endIdx; ++i) {
        const Vector2D& res = GenSolution(testCases[i]);
        localResults.push_back(res);
    }
    
    // 将本地结果合并到全局结果数组
    std::lock_guard<std::mutex> lock(resultsMutex);
    for (int i = startIdx; i < endIdx; ++i) {
        results[i] = localResults[i - startIdx];
    }
}

// 使用线程池处理所有测试用例
void ProcessAllTestCasesParallel() {
    std::vector<Vector2D> results(m);
    std::mutex resultsMutex;
    std::vector<std::thread> threads;
    
    // 计算每个线程处理的测试用例数量
    int casesPerThread = m / NUM_THREADS;
    int remainingCases = m % NUM_THREADS;
    
    int currentStart = 0;
    for (int t = 0; t < NUM_THREADS; ++t) {
        int currentEnd = currentStart + casesPerThread;
        if (t < remainingCases) {
            currentEnd++;
        }
        
        if (currentStart < currentEnd) {
            threads.emplace_back(ProcessTestCaseRange, currentStart, currentEnd, 
                               std::ref(results), std::ref(resultsMutex));
        }
        currentStart = currentEnd;
    }
    
    // 等待所有线程完成
    for (auto& thread : threads) {
        thread.join();
    }
    
    // 按顺序输出结果
    for (int i = 0; i < m; ++i) {
        cout << fixed << std::setprecision(5) << results[i].x << " " << results[i].y << endl;
        cout.flush();
    }
}

void PreProcess()
{
    // 预计算多边形1的边法线、自身投影和中心点
    polygon1.PrecomputeAxes();
    polygon1.PrecomputeSelfProjections();
    polygon1.PrecomputeCenter();
    
    // 预计算多边形2的边法线、自身投影和中心点
    polygon2.PrecomputeAxes();
    polygon2.PrecomputeSelfProjections();
    polygon2.PrecomputeCenter();

    // 预计算交叉投影：多边形1在多边形2的法线上的投影
    polygon1.PrecomputeOtherProjections(polygon2.axes);
    // 预计算交叉投影：多边形2在多边形1的法线上的投影
    polygon2.PrecomputeOtherProjections(polygon1.axes);
}

int main()
{
    // =============== 1. read polygons ===================
    cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        cerr << "Input data error: the number of vertices of both polygons should be greater than 2" << endl;
        return 1;
    }

    polygon1.vertices.resize(n1);
    for (int i = 0; i < n1; ++i) {
        cin >> polygon1.vertices[i].x >> polygon1.vertices[i].y;
    }

    polygon2.vertices.resize(n2);
    for (int i = 0; i < n2; ++i) {
        cin >> polygon2.vertices[i].x >> polygon2.vertices[i].y;
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
    ProcessAllTestCasesParallel();

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