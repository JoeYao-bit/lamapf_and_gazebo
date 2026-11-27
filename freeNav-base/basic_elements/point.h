//
// Created by yaozhuo on 2022/1/2.
//

#ifndef FREENAV_BASE_POINT_H
#define FREENAV_BASE_POINT_H


#define EPS_FR 1e-3


#include <vector>
#include <iostream>
#include <math.h>
#include <set>
#include <limits>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <assert.h>
#include <memory>

#include "../basic_elements/fraction_imported.h"

namespace freeNav {

    typedef unsigned uint;

    typedef unsigned NodeId;
    typedef std::vector<NodeId> NodeIds;

    typedef unsigned EdgeId;
    typedef std::vector<EdgeId> EdgeIds;

    typedef unsigned long long int Id;

    typedef Id DimensionLength;

    typedef unsigned Dimension;

    typedef int Lv;

    typedef long long int Time;

    const Time MIN_TIME = 0;

    template<Dimension N>
    std::string printDimInfo(DimensionLength* dim) {
        std::stringstream ss;
        for(int i=0; i<N; i++) {
            ss << dim[i] << " ";
        }
        return ss.str();
    }

    template<typename KEY>
    KEY MAX = std::numeric_limits<KEY>::max();

    template<typename KEY>
    KEY MIN = std::numeric_limits<KEY>::min();

    typedef std::vector<uint> UintVector;
    typedef std::vector<Id> IdVector;
    typedef std::set<Id> IdSet; // O(log(n)) find time complexity
    //typedef std::unordered_set<Id> IdSet; // O(1) find complexity

    template <Dimension N>
    DimensionLength getMinimumDimension(DimensionLength* dim) {
        DimensionLength min_dim = MAX<DimensionLength>;
        for(int i=0; i<N; i++) {
            if(min_dim > dim[i]) {
                min_dim = dim[i];
            }
        }
        return min_dim;
    }

    // Point to store surface point information
    template<typename T, Dimension N>
    struct Point {

        Point(const Point& pt) {
            for(int i=0; i<N; i++) {
                val_[i] = pt[i];
            }
        }

        Point(std::initializer_list<T> list) {
            assert(list.size() == N);
            for(auto iter = list.begin(); iter != list.end(); iter++) {
                val_[iter-list.begin()] = *iter;
            }
        }

        Point() {
            for (uint i = 0; i < N; i++) {
                val_[i] = 0;
            }
        }

        Point operator-(const Point &pt) const {
            Point re;
            for (uint i = 0; i < N; i++) {
                re[i] = val_[i] - pt[i];
            }
            return re;
        }

        void setAll(const T& val) {
            for (uint i = 0; i < N; i++) {
                val_[i] = val;
            }
        }

        Point operator+(const Point &pt) const {
            Point re = *this;
            for (uint i = 0; i < N; i++) {
                re[i] = val_[i] + pt[i];
            }
            return re;
        }

        std::string toStr() const {
            std::stringstream ss;
            for(uint i = 0; i < N; i++) {
                ss << val_[i] << " ";
            }
            return ss.str();
        }

        T operator*(const Point<int, N> &pt) const {
            T val(0);
            for (uint i = 0; i < N; i++) {
                val += val_[i] * pt[i];
            }
            return val;
        }

        Point<T, N> operator*(const Point<Fraction, N> &pt) const {
            Point<T, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] += val_[i] * pt[i];
            }
            return val;
        }

        Point<Fraction, N> operator/(const Fraction &f) const {
            Point<Fraction, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] += val_[i] / f;
            }
            return val;
        }

        Point<T, N> operator/(const int &f) const {
            Point<T, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] += val_[i] / f;
            }
            return val;
        }

        T maxDim() {
            T maximum = MIN<T>;
            for(int dim=0; dim<N; dim++) {
                if(maximum < val_[dim]) maximum = val_[dim];
            }
            return maximum;
        }

        double operator*(const Point<double, N> &pt) const {
            double val(0);
            for (uint i = 0; i < N; i++) {
                val += val_[i] * pt[i];
            }
            return val;
        }

        float operator*(const Point<float, N> &pt) const {
            float val(0);
            for (uint i = 0; i < N; i++) {
                val += val_[i] * pt[i];
            }
            return val;
        }

        Point<double, N> multi(const double f) const {
            Point<double, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = val_[i] * f;
            }
            return val;
        }

        Point<T, N> multi(const int f) const {
            Point<T, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = val_[i] * f;
            }
            return val;
        }

        Point<T, N> multi(const Point<T, N>& pt) const {
            Point<T, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = val_[i] * pt[i];
            }
            return val;
        }

        bool operator==(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] != pt[i]) return false;
            }
            return true;
        }

        bool operator>(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] <= pt[i]) return false;
            }
            return true;
        }

        bool operator<(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] >= pt[i]) return false;
            }
            return true;
        }

        bool operator>=(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] < pt[i]) return false;
            }
            return true;
        }

        bool operator<=(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] > pt[i]) return false;
            }
            return true;
        }

        void operator=(const Point<T, N-1> &pt) {
            for (uint i = 0; i < N-1; i++) {
                val_[i] = pt[i];
            }
        }

        bool operator!=(const Point &pt) const {
            for (uint i = 0; i < N; i++) {
                if (val_[i] != pt[i]) return true;
            }
            return false;
        }

        double Norm() const {
            double norm(0.);
            for (uint i = 0; i < N; i++) {
                norm += val_[i]*val_[i];
            }
            return sqrt(norm);
        }

        T manhattan() const {
            T dist;
            for (uint i = 0; i < N; i++) {
                dist += abs(val_[i]);
            }
            return dist;
        }

        Point<double, N> Normalize() {
            double length = Norm();
            Point<double, N> pt;
            for (uint i = 0; i < N; i++) {
                pt[i] = val_[i]/length;
            }
            return pt;
        }

        T Square() const {
            T square(0);
            for (uint i = 0; i < N; i++) {
                square += val_[i] * val_[i];
            }
            return square;
        }


        std::ostream &operator<<(std::ostream &os) const {
            os << "(";
            for (int i = 0; i < N - 1; i++) {
                os << val_[i] << ", ";
            }
            os << val_[N-1] << ")";
            return os;
        }

        Point<int, N> ToFloor() const {
            Point<int, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = (int) (floor(val_[i]));
            }
            return val;
        }

        Point<T, N> addPlainOffset(const Point<T, N-1>& plain_offset, int excluded_dim) {
            Point<T, N> retv = *this;
            for(int i=0; i<N; i++) {
                retv[i] = val_[i];
            }
            for(int i=0; i<excluded_dim/2; i++) {
                retv[i] += plain_offset[i];
            }
            for(int i=excluded_dim/2 +1; i<N; i++) {
                retv[i] += plain_offset[i-1];
            }
            return retv;
        }

        Point<int, N> ToCeil() const {
            Point<int, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = (Id) (ceil(val_[i]));
            }
            return val;
        }

        Point<int, N> Round() const {
            Point<int, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = Id(round(val_[i]));
            }
            return val;
        }

        Point<double, N> toDouble() const {
            Point<double, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = val_[i];
            }
            return val;
        }

        Point<float, N> toFloat() const {
            Point<float, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = val_[i];
            }
            return val;
        }

        Point<int, N> toInt() const {
            Point<int, N> val;
            for (uint i = 0; i < N; i++) {
                val[i] = round(val_[i]);
            }
            return val;
        }

        void insert(T val, unsigned int index) {
            std::vector<T>::insert(this->begin() + index, val);
        }

        T& operator[](int index) {
            return val_[index];
        }

        const T& operator[](int index) const {
            return val_[index];
        }

    private:

        T val_[N];
    };


    template<typename T, Dimension N> using Points = std::vector<Point<T, N>>;
    template <Dimension N> using Pointi = Point<int, N>;
    template <Dimension N> using PointF = Point<Fraction, N>;
    template <Dimension N> using Pointf = Point<float, N>;
    template <Dimension N> using Pointd = Point<double, N>;
    template <Dimension N> using Neightbor = std::vector<Pointi<N>>;
    template <Dimension N> using Pointis = std::vector<Pointi<N>>;
    template <Dimension N> using Pointfs = std::vector<Pointf<N>>;
    template <Dimension N> using Pointds = std::vector<Pointd<N>>;



    template <typename T>
    double getAngle(const Point<T, 2>& pt) {
        double retv = acos(pt[0]/pt.Norm());
        return pt[1] > 0 ? retv : -retv;
    }

    typedef float PathLen;

    template <Dimension N>
    using Path = std::vector<Pointi<N> >;

    template <Dimension N>
    using Paths = std::vector<Path<N> >;

    template <Dimension N>
    PathLen calculatePathLength(const Path<N>& path) {
        if(path.size() <= 1) return 0;
        PathLen retv = 0;
        for(int i=0; i<path.size()-1; i++) {
            retv += (path[i+1] - path[i]).Norm();
        }
        return retv;
    }

    template<typename T, Dimension N>
    std::ostream& operator<<(std::ostream& os, const Point<T, N>& pt) {
        os << "(";
        for(int i=0; i<N-1; i++) {
            os << pt[i] <<", ";
        }
        os << pt[N-1] << ")";
        return os;
    }

    template<typename T, Dimension N>
    Point<T, N> operator*(const Point<T, N>& pt, const T& t) {
        Point<T, N> val;
        for (uint i = 0; i < N; i++) {
            val[i] = pt[i] * t;
        }
        return val;
    }

    template<typename T, Dimension N>
    Point<T, N> operator*(const T& t, const Point<T, N>& pt) {
        Point<T, N> val;
        for (uint i = 0; i < N; i++) {
            val[i] = pt[i] * t;
        }
        return val;
    }

    template<Dimension N>
    std::ostream& operator<<(std::ostream& os, const Path<N>& path) {
        if(path.empty()) return os;
        for(int i=0; i<path.size()-1; i++) {
            os << path[i] << "->";
        }
        os << path.back();
        return os;
    }

    template <typename X, typename Y, typename Z, Dimension N>
    static double areaOfThreePoints(const Point<X, N>& pt1, const Point<Y, N>& pt2, const Point<Z, N>& pt3) {
        double d1 = (pt1 - pt2).Norm(), d2 = (pt2 - pt3).Norm(), d3 = (pt1 - pt3).Norm();
        double p = (d1 + d2 + d3)/2.;
        return sqrt(p*(p-d1)*(p-d2)*(p-d3));
    }

    template <typename T, Dimension N>
    static bool angleNotMinorThan90(const Point<T, N>& pt1, const Point<T, N>& pt2, const Point<T, N>& pt3) {
        T flag = pt2*pt2 - pt2*pt1 - pt2*pt3 + pt1*pt3;
        return flag <= 0;
    }

    template <typename T, Dimension N>
    static double angleBetweenThreePoints(const Point<T, N>& pt1, const Point<T, N>& pt2, const Point<T, N>& pt3) {
        if(pt1 == pt2 || pt1 == pt3) return 0;
        double numerator = pt2*pt2 - pt2*pt1 - pt2*pt3 + pt1*pt3;
        double denominator = (pt2-pt1).Norm() * (pt2-pt3).Norm();
        return acos(numerator / denominator);
    }

    template <typename X, typename Y, typename Z, Dimension N>
    static double pointDistToLine(const Point<X, N>& pt, const Point<Y, N>& line1, const Point<Z, N>& line2) {
        double area = areaOfThreePoints(pt, line1, line2);
        double base_length = (line1-line2).Norm();
        double d1 = (line1-pt).Norm(), d2 = (line2 - pt).Norm(), d3 = (line1 - line2).Norm();
        double cos1 = pow(d1, 2) + pow(d3, 2) - pow(d2, 2);
        double cos2 = pow(d2, 2) + pow(d3, 2) - pow(d1, 2);
        if(cos1 < 0) {
            return d1;
        } else if (cos2 < 0) {
            return d2;
        }
        return 2*area / base_length;
    }

    template <typename X, typename Y, typename Z, Dimension N>
    static double pointDistToLineNoLengthLimit(const Point<X, N>& pt, const Point<Y, N>& line1, const Point<Z, N>& line2) {
        double area = areaOfThreePoints(pt, line1, line2);
        double base_length = (line1-line2).Norm();
        return 2*area / base_length;
    }

    // no float calculation
    template <Dimension N>
    static bool pointDistToLineNoLengthLimitNotMinorThan1(const Pointi<N>& pt1, const Pointi<N>& pt2, const Pointi<N>& pt3) {
        Pointi<N> line12 = (pt1 - pt2), line23 = (pt3 - pt2);
        int pt12 = line12.Square(), pt23 = line23.Square();
        int pt2_pt1 = line12 * line23;
        return pt12 * pt23 - pt2_pt1 * pt2_pt1 >= pt23;
    }

    template <Dimension N>
    static bool PointOnLine(const Pointi<N>& pt1, const Pointi<N>& pt2, const Pointi<N>& pt3) {
        Pointi<N> line1 = pt1 - pt2, line2 = pt1 - pt3;
        int val1 = line1 * line2;
        return val1 <=0 && pow(val1, 2) == line1.Square() * line2.Square();
    }

    // check whether line pt1-pt2 cross line pt3-pt4
    // if one of the end of line on another line, its cross
    template <Dimension N>
    static bool IsLineCrossLine(const Pointi<N>& pt1, const Pointi<N>& pt2, const Pointi<N>& pt3, const Pointi<N>& pt4) {
        if(PointOnLine(pt1, pt3, pt4)) return true;
        if(PointOnLine(pt2, pt3, pt4)) return true;
        if(PointOnLine(pt3, pt1, pt2)) return true;
        if(PointOnLine(pt4, pt1, pt2)) return true;

        Pointi<N> line14 = (pt1 - pt4), line24 = (pt2 - pt4),
               line13 = (pt1 - pt3), line23 = (pt2 - pt3);
        double a134 = angleBetweenThreePoints(pt1, pt3, pt4), a234 = angleBetweenThreePoints(pt2, pt3, pt4), a132 = angleBetweenThreePoints(pt1, pt3, pt2);
        bool flag1 = fabs(a134 + a234 - a132) < 1e-3;
        double a143 = angleBetweenThreePoints(pt1, pt4, pt3), a243 = angleBetweenThreePoints(pt2, pt4, pt3), a142 = angleBetweenThreePoints(pt1, pt4, pt2);
        bool flag2 = fabs(a143 + a243 - a142) < 1e-3;
        return flag1 && flag2;
    }



    template <Dimension N>
    PointF<N> PointiToPointF(const Pointi<N> & pt)  {
        PointF<N> rpt;
        for(int i=0; i<N; i++) {
            rpt[i] = Fraction(pt[i]);
        }
        return rpt;
    }

    template <Dimension N>
    PointF<N> GetFootOfPerpendicular(
            const Pointi<N> &pt,     // 直线外一点
            const Pointi<N> &begin,  // 直线开始点
            const Pointi<N> &end)   // 直线结束点
    {
        PointF<N> retVal;
        PointF<N> delta = PointiToPointF(begin - end);
        Fraction dist_2_line = delta.Square();
        auto u = Fraction((pt - begin) * (begin - end));
        u = u / dist_2_line;
        retVal = PointiToPointF(begin) + u * delta;
        return retVal;
    }

    template <Dimension N>
    PointF<N> getPerpedicularPoint(const Pointi<N>& pt1, const Pointi<N>& pt2, const Pointi<N>& x) {
        PointF<N> p2p3 = PointiToPointF(x - pt2);
        PointF<N> p2p1 = PointiToPointF(pt2 - pt1);
        // use fraction for replace
        PointF<N> BD = p2p3 * p2p1 / ((pt2 - x).Square()) * p2p3;
        PointF<N> p4 = PointiToPointF(pt2) - BD;
        return p4;
    }

    // Line is temporary data type, so no access to space are contained
    template <typename T, Dimension N>
    struct Line
    {
        // construct a line that connect the two points
        // the first line is first add to the path
        // first and second CANNOT be the same point
        explicit Line(const Point<T, N> &first, const Point<T, N> &second) {
            if(first==second) {
                exit(0);
            }
            T devia(0);
            for(uint i=0; i<N; i++) {
                if(devia < abs(second[i] - first[i])) {
                    devia = abs(second[i] - first[i]);
                    start = first[i];
                    end = second[i];
                    dimension_index = i;
                }
            }
            step = devia;
            start < end ? step_length = 1 : step_length = -1;
            parameter.resize(N);
            for(unsigned long int i=0; i<parameter.size(); i++) {
                parameter[i].first = first[i]; // numerator
                parameter[i].second = second[i] - first[i]; // denominator
            }
        }

        // get a point one the line
        // index : distant to the first point
        // index MUST be minus than step
        Pointi<N> GetPoint(Id index) {
            double base = start + index * step_length;
            Pointi<N> pt;
            // y = k*x + b -> line equation
            double k = (base - parameter[dimension_index].first)/parameter[dimension_index].second;
            for(uint i=0; i<parameter.size(); i++) {
                pt[i] = round(parameter[i].second*k + parameter[i].first);
            }
            return pt;
        }

        Pointf<N> GetPointFloat(Id index) {
            double base = start + index * step_length;
            Pointf<N> pt;
            // y = k*x + b -> line equation
            double k = (base - parameter[dimension_index].first)/parameter[dimension_index].second;
            for(uint i=0; i<parameter.size(); i++) {
                pt[i] = parameter[i].second*k + parameter[i].first;
            }
            return pt;
        }

        PointF<N> GetPointF(Id index)  {
            Fraction base = start + index * step_length;
            PointF<N> pt;
            // y = k*x + b -> line equation
            Fraction k = (base - parameter[dimension_index].first)/parameter[dimension_index].second;
            for(uint i=0; i<parameter.size(); i++) {
                pt[i] = parameter[i].second*k + parameter[i].first;
            }
            return pt;
        }

        PointF<N> GetPointF(Fraction index)  {
            Fraction base = start + index * step_length;
            PointF<N> pt;
            // y = k*x + b -> line equation
            Fraction k = (base - parameter[dimension_index].first)/parameter[dimension_index].second;
            for(uint i=0; i<parameter.size(); i++) {
                pt[i] = parameter[i].second*k + parameter[i].first;
            }
            return pt;
        }

        Pointd<N> GetPointd(Id index) {
            double base = start + index * step_length;
            Pointd<N> pt(parameter.size());
            // y = k*x + b -> line equation
            double k = (base - parameter[dimension_index].first)/parameter[dimension_index].second;
            for(uint i=0; i<parameter.size(); i++) {
                pt[i] = parameter[i].second*k + parameter[i].first;
            }
            return pt;
        }


        // from which dimension start Bresham algorithm (light projection)
        unsigned int dimension_index;
        // start < end
        long long int start, end; // how many grid the line will cross
        long long int step; // step = DistBetween(first, second)/(end - start)
        int step_length;
        // parameter of point oblique line equation, first for numerator while second for denominator
        std::vector<std::pair<long long int, long long int> > parameter;
    };

    template <Dimension N>
    using IS_OCCUPIED_FUNC = std::function<bool(const Pointi<N>&)>;

    // if the line is collision free, return true, otherwise false
    template <typename T, Dimension N>
    using IS_LINE_COLLISION_FREE_FUNC = std::function<bool(const Point<T, N>&, const Point<T, N>&)>;

    template <Dimension N>
    using SET_OCCUPIED_FUNC = std::function<void(const Pointi<N>&)>;

    template<Dimension N>
    Pointi<N> toAbs(const Pointi<N>& val) {
        Pointi<N> pt;
        for (uint i = 0; i < N; i++) {
            pt[i] = abs(val[i]);
        }
        return pt;
    }

    template <Dimension N>
    Id getTotalIndexOfSpace(DimensionLength* dimension_info) {
        Id total_index = 1;
        for(int i = 0; i < N; i++) {
            total_index *= dimension_info[i];
        }
        return total_index;
    }

    template <Dimension N>
    int getTotalIndexOfSpace(const Pointi<N>& pt) {
        int total_index = 1;
        for(int i = 0; i < N; i++) {
            total_index *= pt[i];
        }
        return total_index;
    }

    // there are 2*N boundary surface for a N dimension grid space
    template <Dimension N>
    Id getTheCountOfBoundaryGrid(DimensionLength* dimension_info) {
        Id total_count = 0;
        for(int i = 0; i < N; i++) {
            Id current_plain_grid_count = 1;
            for(int j = i+1; j < N; j++) {
                current_plain_grid_count = current_plain_grid_count*dimension_info[j];
            }
            total_count = total_count + 2*current_plain_grid_count;
        }
        return total_count;
    }

    /* wrapper of is_occupied that determine whether pt1 -> pt2 is collision free */
//    template <Dimension N>
//    bool LineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2, bool (*is_occupied)(const Pointi<N>& pt));

    template <Dimension N>
    Pointis<N> lineTravelsal(const Pointi<N>& pt1, const Pointi<N>& pt2);


    template <Dimension N>
    Pointi<N> IdToPointi(const Id &index, DimensionLength* dimension_info) {
        Pointi<N> pt;
        pt[0] = index%dimension_info[0];
        DimensionLength buff(index/dimension_info[0]);
        for(Id i=1; i<N; i++) {
            pt[i] = buff%dimension_info[i];
            buff = buff/dimension_info[i];
        }
        return pt;
    }

    template <Dimension N>
    Id PointiToId(const Pointi<N> &pt, const DimensionLength* dimension_info) {
        Id index(pt[0]);
        Id buff(dimension_info[0]);
        for(unsigned long int i=1; i<N; i++) {
            index += pt[i]*buff;
            buff *= dimension_info[i];
        }
        return index;
    }

    // the nearest 2*N grids
    template <Dimension N>
    Pointis<N> GetNearestOffsetGrids() {
        Pointis<N> near_offset;
        Pointi<N> buff;
        for(uint i=0; i<N; i++) {
            buff[i] += 1;
            near_offset.push_back(buff);
            buff[i] -= 2;
            near_offset.push_back(buff);
            buff[i] += 1;
        }
        near_offset.shrink_to_fit();
        return near_offset;
    }

    template<Dimension N>
    Pointi<N> ToNdPoint(Id index, Id width) {
        Pointi<N> pt;
        for(int i=0; i<N; i++) {
            pt[i] = index%width - (width-1)/2;
            index = index/width;
        }
        return pt;
    }

    // nearest 3^n grids
    template <Dimension N>
    Pointis<N> GetNeightborOffsetGrids(bool with_center = false) {
        Pointis<N> neightbor_offset;
        for(unsigned long int i=0; i<pow(3, N); i++) {
            Pointi<N> ndp = ToNdPoint<N>(i, 3);
            if(!with_center && ndp == Pointi<N>()) continue;
            neightbor_offset.push_back(ndp);
        }
        neightbor_offset.shrink_to_fit();
        return neightbor_offset;
    }

    template <Dimension N>
    std::vector<Pointis<N> > initAllDirectionLocalMoves() {
        Pointis<N-1> next_plain = GetNeightborOffsetGrids<N-1>(true);
        std::vector<Pointis<N> > retv;
        for(int dim=0; dim<2*N; dim++) {
            Pointi<N> current_pt;
            Pointis<N> current_surface;
            for(const auto& next_plain_pt : next_plain) {
                for(int i=0; i<N; i++) {
                    if(i == dim/2) {
                        current_pt[i] = dim % 2 == 0 ? 1 : -1;
                    } else if(i > dim/2) {
                        current_pt[i] = next_plain_pt[i-1];
                    } else {
                        current_pt[i] = next_plain_pt[i];
                    }
                }
                current_surface.push_back(current_pt);
            }

            retv.push_back(current_surface);
        }
        return retv;
    }

    template <Dimension N>
    Pointis<N> GetFloorOrCeilFlag() {
        Pointis<N> floor_or_ceil;
        for(unsigned long int i=0; i<pow(2, N); i++) {
            Pointi<N> ndp = ToNdPoint<N>(i, 2);
            floor_or_ceil.push_back(ndp);
        }
        floor_or_ceil.shrink_to_fit();
        return floor_or_ceil;
    }

    template <Dimension N>
    Pointis<N> GetSphereInflationOffsetGrids(uint inflation_radiu) {
        Pointis<N> buff;
        Pointis<N> inflation_offset;
        for(unsigned long int i=0; i<pow(2*inflation_radiu+1, N); i++) {
            Pointi<N> ndp = ToNdPoint<N>(i, 2*inflation_radiu+1);
            if(ndp == Pointi<N>()) continue;
            buff.push_back(ndp);
        }
        buff.shrink_to_fit();
        Pointi<N> zero;
        for(const Pointi<N>& offset : buff) {
            if(DistBetween(offset, zero) <= inflation_radiu)
                inflation_offset.push_back(offset);
        }
        inflation_offset.shrink_to_fit();
        return inflation_offset;
    }

    // distant between two points
    template <Dimension N>
    double DistBetween(const Pointi<N>& p1, const Pointi<N>& p2) {
        Pointi<N> pt = p1 - p2;
        return pt.Norm();
    }

    template <Dimension N>
    Pointi<N> PointFloatToInteger(const Pointd<N>& ptd, const Pointi<N>& fc) {
        Pointi<N> val;
        for(uint i=0; i<N; i++) {
            if(fc[i]==0) val[i] = ceil(ptd[i]);
            else val[i] = floor(ptd[i]);
        }
        return val;
    }

    template <Dimension N>
    bool ObstacleNearbyCheck(const Pointi<N>& pt,
                             const Line<int, N>& line,
                             const Pointi<N> &first,
                             const Pointi<N> &second,
                             IS_OCCUPIED_FUNC<N> is_occupied,
                             const Pointis<N-1>& neighbor) {
        Pointi<N> temp_pt;
        for(const auto& offset : neighbor) {
            temp_pt = pt;
            for(int i=0; i<N; i++) {
                if(i < line.dimension_index) {
                    temp_pt[i] += offset[i];
                } else if(i > line.dimension_index) {
                    temp_pt[i] += offset[i-1];
                }
            }
            if(!is_occupied(temp_pt)) {
                continue;
            }
            if (!pointDistToLineNoLengthLimitNotMinorThan1(temp_pt, first, second)) {
                return true;
            }
        }
        return false;
    }

    template <Dimension N>
    bool LineCrossInCubic(const Pointi<N>& pt,
                          const Pointi<N>& pt1, const Pointi<N>& pt2,
                          const Line<int, N>& line,
                          const Pointis<N-1>& neighbor,
                          IS_OCCUPIED_FUNC<N> is_occupied
                          ) {
        Pointi<N> temp_pt;
        for(const auto& offset : neighbor) {
            temp_pt = pt;
            for(int i=0; i<N; i++) {
                if(i < line.dimension_index) {
                    temp_pt[i] += offset[i];
                } else if(i > line.dimension_index) {
                    temp_pt[i] += offset[i-1];
                }
            }
            if(is_occupied(temp_pt))
            {
                //
                // get perpendicular point p' from pt to line
                // if all coordinate of p' in the cubic of occ, line cross occupied
                PointF<N> pdp = GetFootOfPerpendicular(temp_pt, pt1, pt2);
                PointF<N> occ = PointiToPointF(temp_pt);
                PointF<N> offset_f = (pdp - occ);
                Fraction half(1, 2);
                //std::cout << "occ temp_pt = " << temp_pt << std::endl;
                bool occed = true;
                for(int i=0; i<N; i++) {
                    if((offset_f[i].toAbs()) >= half) { occed = false; }
                }
                if(occed) { return true; }
            }
        }
        return false;
    }

    // TODO: use quadTree map for acceleration
    template <Dimension N>
    bool LineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2, IS_OCCUPIED_FUNC<N> is_occupied, const Pointis<N-1>& neighbor) {
        if(pt1 == pt2) return true;
        Line<int, N> line(pt1, pt2);
        int check_step = line.step;
        Pointi<N> pt;
        for(int i=1; i<check_step; i++) {
            pt = line.GetPoint(i);
            if(is_occupied(pt)) {
                return true;
            }
        }
        return false;
    }

    /* does work like a continues way */
    template <typename T, Dimension N>
    Points<T, N> lineTraversal(const Point<T, N>& pt1, const Point<T, N>& pt2) {
        Points<T, N> retv;
        if(pt1 == pt2) {
            retv.push_back(pt1);
            return retv;
        }
        Line<int, N> line(pt1, pt2);
        Pointis<N> neighbor = GetNeightborOffsetGrids<N>();
        int check_step = line.step;
        retv.push_back(pt1);
        for(int i=1; i<check_step; i++) {
            Pointis<N> nearby_for_check;
            Pointi<N> pt = line.GetPoint(i);
            retv.push_back(pt);
        }
        retv.push_back(pt2);
        retv.shrink_to_fit();
        return retv;
    }

    template <Dimension N>
    Pointd<N> projectToLine(const Pointi<N>& pt1, const Pointi<N>& line1, const Pointi<N>& line2) {
        double line_length = (line1 - pt1).Norm();
        Pointd<N> unit_vector = (line2 - line1).Normalize();
        Pointd<N> project = line1.toDouble() + unit_vector.multi(cos(angleBetweenThreePoints(pt1, line1, line2)) * line_length);
        return project;
    }

    bool isOneInFourCornerOccupied(const FractionPair& start_pt, IS_OCCUPIED_FUNC<2> is_occupied);

    bool isOneInFourCornerOccupied(const FractionPair& start_pt, IS_OCCUPIED_FUNC<2> is_occupied);

    template <Dimension N>
    bool isOutOfBoundary(const Pointi<N>& pt, DimensionLength* dimension_info) {
        for(int i=0; i<N; i++) {
            if(pt[i] < 0 || pt[i]>= dimension_info[i]) return true;
        }
        return false;
    }

    template<Dimension N>
    Pointis<N> getAllGridInsideASpace(DimensionLength* block_dimen) {
        Id total_index_block = getTotalIndexOfSpace<N>(block_dimen);
        Pointis<N> block_pts; Pointi<N> block_pt;
        // get all grid of the cubic
        for(int j=0; j<total_index_block; j++) {
            block_pt = IdToPointi<N>(j, block_dimen);
            block_pts.push_back(block_pt);
        }
        return block_pts;
    }

    template<Dimension N>
    bool isSamePath(const Path<N>& p1, const Path<N>& p2)
    {
        if (p1.size() != p2.size())
            return false;
        for (unsigned i = 0; i < p1.size(); i++)
        {
            if (p1[i] != p2[i])
                return false;
        }
        return true;
    }

    template <typename T, Dimension N>
    struct Pose {
        Pose(Point<T, N> pt, int orient) : pt_(pt), orient_(orient) {}

        Pose() : pt_(Pointi<N>()), orient_(0) {}

        Point<T, N> pt_;

        // for a N dimensional space, there are 2*N orientation
        // e.g., 0,1,2,3 for 2D, 0,1,2,3,4,5,6,7 for 3D
        // int orient are equal to a orthogonal vector:
        // e.g., 2D: 0 -> (1, 0),    1 -> (-1, 0),    2 -> (0, 1),    3 -> (0, -1)
        //       3D: 0 -> (1, 0, 0), 1 -> (-1, 0, 0), 2 -> (0, 1, 0), 3 -> (0, -1, 0), 4 -> (0, 0, 1), 5 -> (0, 0, -1),
        int orient_ = 0;

    };

    template <typename T, Dimension N>
    std::ostream & operator<<(std::ostream &out, const Pose<T, N>& pose) {
        out << pose.pt_ << "[" << pose.orient_ << "]";
        return out;
    }
    template <typename T, Dimension N>
    using PosePtr = std::shared_ptr<Pose<T, N> >;

    template <Dimension N>
    using Instance = std::pair<Pointi<N>, Pointi<N> >;

    template <Dimension N>
    using InstanceOrient = std::pair<Pose<int, N>, Pose<int, N> >;

    template <Dimension N>
    using Instances = std::vector<Instance<N> >;

    template <Dimension N>
    using InstanceOrients = std::vector<InstanceOrient<N> >;

    template <Dimension N>
    using InstancesS = std::vector<Instances<N> >;

    template <Dimension N>
    using InstanceOrientsS = std::vector<InstanceOrients<N> >;

    template <Dimension N>
    std::ostream& operator << (std::ostream& os, const Instances<N>& ists) {
        for(const Instance<N>& ist : ists) {
            os << "(" << ist.first << "->" << ist.second << ")";
        }
        return os;
    }

    template <typename T>
    std::ostream& operator << (std::ostream& os, const std::set<T>& set_of_t) {
        for(const auto& t : set_of_t) {
            os << t << " ";
        }
        return os;
    }

    // interfaces for layered MAPF
    using VERTEX_CONSTRAINED_FUNC = std::function<bool(const int& loc, const int& next_time)>;

    using EDGE_CONSTRAINED_FUNC = std::function<bool(const int& from, const int& to, const int& next_time)>;

    using HOLDING_TIME_FUNC = std::function<int(const int& loc, const int& earliest_time)>;

    using GET_MAX_TIME_FUNC = std::function<int()>;

    struct GENERAL_CONSTRAINT_TABLE {

        explicit GENERAL_CONSTRAINT_TABLE(const VERTEX_CONSTRAINED_FUNC& vertex_func,
                                          const EDGE_CONSTRAINED_FUNC& edge_func,
                                          const HOLDING_TIME_FUNC& holding_func = HOLDING_TIME_FUNC(),
                                          const GET_MAX_TIME_FUNC& max_time_func = GET_MAX_TIME_FUNC()) :
                vertex_func_(vertex_func), edge_func_(edge_func),
                holding_func_(holding_func), max_time_func_(max_time_func) {}

        const VERTEX_CONSTRAINED_FUNC& vertex_func_;
        const EDGE_CONSTRAINED_FUNC& edge_func_;
        const HOLDING_TIME_FUNC& holding_func_;
        const GET_MAX_TIME_FUNC& max_time_func_;
    };


}

#endif //FREENAV_POINT_H
