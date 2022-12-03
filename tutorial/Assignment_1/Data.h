#pragma once
#include <utility>
#include <min_heap.h>
#include <Eigen/Core>
#include <set>

using namespace std;
using namespace Eigen;
using namespace igl;
class Data {
public:
    Data(MatrixXd OV, MatrixXi OF) : OV(OV), OF(OF), V(OV), F(OF), index(0), current_available_collapses(1), manual_reset_selected(false) { };
    VectorXi EMAP;
    MatrixXi F, E, EF, EI;
    VectorXi EQ;
    MatrixXd V, C;
    MatrixXi OF;
    MatrixXd OV;
    MatrixXd VN, FN, T;
    min_heap<tuple<double, int, int>> original_Q;
    typedef set<pair<double, int>> PriorityQueue;
    PriorityQueue new_Q; // priority Queue: edge vs cost
    vector<PriorityQueue::iterator> Q_iter;
    vector <Matrix4d> Q_matrix; //Vector: vertex vs Q matrix
    int num_collapsed;
    int index;
    int current_available_collapses;
    bool manual_reset_selected;
private:
    // none
};
