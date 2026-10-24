/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mocbs_nb.hpp"
#include <fstream>
#include <sstream>
#include <map>

// #define __MY_COMPILE_KUNG_SORT__ false // for comparison, should never be used in practice.
#define __MY_COMPILE_KUNG_SORT__ true

// #define __MY_COMPILE_NONDOM_SORT__ false // for comparison, should nenver bve used in practice.
#define __MY_COMPILE_NONDOM_SORT__ true

#include <cstdlib>
#include <iostream>
#include <queue>
#include <tuple>
#include <unordered_set>

namespace rzq{
namespace mapf{

static std::map<int, std::vector<long> > g_target;
static bool TgtOK(const std::vector<CBSConstraint*>& cs, int ri) {
  auto it = g_target.find(ri);
  if (it == g_target.end()) return false;
  const std::vector<long>& p = it->second;
  auto at = [&](long t) { return t < (long)p.size() ? p[t] : p.back(); };
  for (auto c : cs) {
    if (!c->IsValid()) continue;
    if (c->IsNodeCstr()) { if (at(c->ta) == c->vi) return false; }
    else if (c->IsEdgeCstr()) { if (at(c->ta) == c->vi && at(c->ta + 1) == c->vj) return false; }
  }
  return true;
}

bool LexLess(const basic::CostVector& a, const basic::CostVector& b) {
  size_t d = std::min(a.size(), b.size());
  for (size_t i = 0; i < d; i++) {
    if (a[i] < b[i]) return true;
    if (a[i] > b[i]) return false;
  }
  return a.size() < b.size();
}

basic::CostVector LexMin(const basic::CostVector& a, const basic::CostVector& b) {
  return LexLess(a, b) ? a : b;
}

basic::CostVector LexMax(const basic::CostVector& a, const basic::CostVector& b) {
  return LexLess(a, b) ? b : a;
}

void NonDomVectors(const std::vector<basic::CostVector>& in, std::vector<basic::CostVector>& out) {
  out.clear();
  for (size_t i = 0; i < in.size(); i++) {
    bool dominated = false;
    for (size_t j = 0; j < in.size(); j++) {
      if (i == j) continue;
      if (search::EpsDominance(in[j], in[i])) {
        if (in[j] == in[i]) {
          if (j < i) {
            dominated = true;
            break;
          }
        } else {
          dominated = true;
          break;
        }
      }
    }
    if (!dominated) {
      out.push_back(in[i]);
    }
  }
}

bool IsCoveredBy(const std::vector<basic::CostVector>& S, const std::unordered_map<long, basic::CostVector>& C) {
  if (S.empty()) return false;
  for (const auto& s : S) {
    bool covered = false;
    for (const auto& c_item : C) {
      if (search::EpsDominance(c_item.second, s)) {
        covered = true;
        break;
      }
    }
    if (!covered) return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& os, const CostIndexCombo& n)
{
  os << "{cvec:" << n.first << ",IDs:(";
  for (auto iy: n.second) {
    os << iy << ",";
  }
  os << ")}";
  return os;
};

int runMinkowskiSum(
  std::unordered_map<long, CostIndexCombo>& in1,
  std::unordered_map<long, basic::CostVector>& in2,
  std::unordered_map<long, CostIndexCombo> *out,
  int mode, int ri)
{
  out->clear();
  int temp_id = -1;
  for (auto item1: in1) {
    for (auto item2: in2) {
      // std::cout << " --- " << item1.second.first << ", " << item2.second << std::endl;
      (*out)[temp_id] = CostIndexCombo();
      out->at(temp_id).first = item1.second.first + item2.second;
      auto index_vec = item1.second.second;

      if (mode==0) {
        index_vec.push_back(item2.first);
      }else if (mode==1) {
        index_vec[ri] = item2.first;
      }else if (mode==2) {
        // do nothing
      }else {
        std::cout << " mode = " << mode << std::endl;
        throw std::runtime_error("[ERROR] runMinkowskiSum(), unknown mode.");
      }
      out->at(temp_id).second = index_vec;
      temp_id--;
    }
  }
  return 1;
};

int nondomSubsetKung(std::unordered_map<long, CostIndexCombo> *out) 
{
  if (out->size() == 0) {return 1;} // MO-CBS probably times out.

  std::list<CostIndexCombo> sorted;
  // lex sort via heap
  std::set< std::pair< basic::CostVector, long> > heap;
  for (auto& item: (*out)) {
    heap.insert( std::make_pair(item.second.first, item.first) );
  }
  while (!heap.empty()) {
    auto cvec = heap.begin()->first;
    long key = heap.begin()->second;
    sorted.push_back(out->at(key));
    heap.erase(heap.begin());
  }

  // compute non-dom subset

  /////  case M=2 ///////
  if (sorted.front().first.size() == 2) { 
    out->clear();
    long temp_id = 1;
    long g2min=1e10;
    for (auto item: sorted) {
      if (item.first[1] >= g2min) {
        // std::cout << " nondomSubsetKung filters " << item << std::endl;
        continue;
      }
      g2min = item.first[1];
      // std::cout << " nondomSubsetKung keeps " << item << " g2min = " << g2min << std::endl;
      (*out)[temp_id] = item;
      temp_id++;
    }
    return 1;
  }

  /////  case M=3 ///////
  if (sorted.front().first.size() == 3) { // M=3
    // This is not a full Kung's method for M=3! I do not implement the AVL-tree but simply use an array.
    // This is not tested yet!!!
    out->clear();
    long temp_id = 1;
    std::list<CostIndexCombo> front; // front is a set of vector of length 2.
    for (auto item: sorted) {
      // check dom for the project vector
      bool dom = false;
      for (auto item2: front) {
        if ( (item2.first[1] <= item.first[1]) && (item2.first[2] <= item.first[2]) ){
          // search::EpsDominance(vecProj(vec), vecProj(item.first)) ){
          dom = true;
          break;
        }
      }
      if (dom) {continue;}

      front.push_back(item);
      (*out)[temp_id] = item;
      temp_id++;      

    }
    return 1;
  }

  ///// case M!=2 and M!=3
  else {
    std::cout << "nondomSubsetKung() is only implemented for M=2,3" << " here M = " << sorted.front().first.size() << std::endl;
    throw std::runtime_error("[ERROR] nondomSubsetKung() M>3 not implemented!");
    return 0;
  }
};

int nondomSubsetNaive(std::unordered_map<long, CostIndexCombo> *out) 
{
  // double for-loop
  if (out->size() == 0) {return 1;} // MO-CBS probably times out.

  std::unordered_set<long> id_to_delete;
  for (auto& item1: (*out)) {
    if (id_to_delete.find(item1.first) != id_to_delete.end()){
      continue;
    }
    auto cvec1 = item1.second.first;
    for (auto& item2: (*out)) {
      if (item2.first == item1.first) {continue;}
      if (id_to_delete.find(item2.first) != id_to_delete.end()){
        continue;
      }
      auto cvec2 = item2.second.first;
      if (search::EpsDominance(cvec1, cvec2)) {
        id_to_delete.insert(item2.first);
      }
    }
  }

  for (auto k : id_to_delete) {
    out->erase(k);
  }
  return 1;
};

void NBCBSNode::computeSortedPF()
{
  // init a combo data structure.
  sorted_costIndex.clear();
  std::unordered_map<long, CostIndexCombo> *combo, *combo2;
  combo = new std::unordered_map<long, CostIndexCombo>();
  long temp_id = 1;
  for (auto& item: idvl_paths[0]) {
    long path_key = item.first;
    (*combo)[temp_id] = CostIndexCombo();
    combo->at(temp_id).first = idvl_costs[0][path_key];
    combo->at(temp_id).second = std::vector<long>({path_key});
    temp_id++;
  }
  for (long ri = 1; ri < idvl_costs.size(); ri++) { // assume there is more than one agent.
    combo2 = new std::unordered_map<long, CostIndexCombo>();
    runMinkowskiSum(*combo, idvl_costs[ri], combo2) ;

    if (__MY_COMPILE_NONDOM_SORT__) { // if this flag is false, all joint paths will be kept for testing purposes.
      if (__MY_COMPILE_KUNG_SORT__){
        nondomSubsetKung(combo2) ;
      }else{
        nondomSubsetNaive(combo2) ;
      }
    }
    
    // std::cout << " a3" << std::endl;
    delete combo;
    combo = combo2;
    combo2 = NULL;
  }

  // now, combo is a pointer to a hash
  // lex sort via heap
  std::set< std::pair< basic::CostVector, long> > heap;
  for (auto& item: (*combo)) {
    heap.insert( std::make_pair(item.second.first, item.first) );
  }
  while (!heap.empty()) {
    auto cvec = heap.begin()->first;
    // std::cout << " heap popped " << cvec << std::endl;
    long key = heap.begin()->second;
    sorted_costIndex.push_back(combo->at(key));
    heap.erase(heap.begin());
  }

  delete combo;
  return ;
};

void NBCBSNode::printPF() {
  int rank = 1;
  for (auto ix : sorted_costIndex) {
    std::cout << " rank:" << rank << ix << std::endl;
    rank++;
  }
  return ;
};


bool NBCBSNode::solFilter(std::unordered_map< long, basic::CostVector >& sol_costs)
{
  bool ret_flag = false;
  while (!sorted_costIndex.empty()) {
    bool dom = false;
    for (auto& item2: sol_costs) {
      if (search::EpsDominance(item2.second, sorted_costIndex.front().first)) {
        dom = true;
        break;
      }
    }
    if (dom) {
      sorted_costIndex.pop_front();
      ret_flag = true;
    }
    else {
      break; // stop at the first non-dom one (Algorithm 3)
    }
  }
  return ret_flag;
};

bool NBCBSNode::setRep()
{
  if (sorted_costIndex.size() == 0) {
    // this node should be pruned.
    return false;
  }else{
    rep_cvec = sorted_costIndex.begin()->first;
    rep_Ids = sorted_costIndex.begin()->second;
    for (int i = 0; i < idvl_costs.size(); i++) {
      rep_jpath[i] = idvl_paths[i][rep_Ids[i]];
      if (rep_jpath[i].size() == 0) {
        throw std::runtime_error("[ERROR] NBCBSNode::setRep zero size path.");
      }
    }
  }
  return true;
};

bool NBCBSNode::isEmptyPlans() {
  return (sorted_costIndex.size() == 0);
};

std::ostream& operator<<(std::ostream& os, const NBCBSNode& n)
{
  os << "NBCBSNode{id:" << n.id << ",parent:" << n.parent << ",cstr:" << n.cstr 
     << ",rep_cvec:" << n.rep_cvec << ",rep_path:" << n.rep_jpath << "}";
  return os;
};


/////////////////////////////////////////////////////////////////////////
////////////////////// NBMOCBS_EMOA /////////////////////
/////////////////////////////////////////////////////////////////////////

NBMOCBS_EMOA::NBMOCBS_EMOA() {
  // std::cout << "[INFO] Constructor - NBMOCBS_EMOA" << std::endl;
};

NBMOCBS_EMOA::~NBMOCBS_EMOA() {};

void NBMOCBS_EMOA::Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) {
  std::cout << "***BB-MO-CBS-H1 search starts***" << std::endl;
  // assign graph pointer here.
  for (int ri = 0; ri < _nAgent; ri++){
    SetGraphPtr(ri, _nAgent, &(_tempAgentGrids[ri]), _agentCosts[ri]);
  }
  // set epsilon, by default 0.
  _epsilon = eps;
  if (getenv("H1_K")) { _K = atoi(getenv("H1_K")); }
  const bool h1_nocover = (getenv("H1_NOCOVER") != NULL);
  if (getenv("H1_TARGET")) {
    std::ifstream fin(getenv("H1_TARGET"));
    std::string line;
    while (std::getline(fin, line)) {
      std::istringstream is(line);
      int a; long v;
      if (!(is >> a)) continue;
      std::vector<long> pth;
      while (is >> v) pth.push_back(v);
      g_target[a] = pth;
    }
    std::cerr << "[TRACE] loaded target for " << g_target.size() << " agents" << std::endl;
  }
  _result.others.resize(2); // to store the average runtime for computing Pareto-optimal front.
  _result.others[0] = 0; // total time
  _result.others[1] = 0; // total number of calls

  // copy
  if (starts.size() != _nAgent) {
    std::cout << "[ERROR] NBMOCBS_EMOA::Search input starts.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] NBMOCBS_EMOA::Search input starts size mismatch!");
  }
  if (goals.size() != _nAgent) {
    std::cout << "[ERROR] NBMOCBS_EMOA::Search input goals.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] NBMOCBS_EMOA::Search input goals size mismatch!");
  }
  _vo = starts;
  _vd = goals;
  _tlimit = time_limit;
  _t0 = std::chrono::steady_clock::now(); // for timing.

  // Line 1-3: Init (P_o, OPEN with g_o,rep, C = empty)
  _Init();
  _result.find_all_pareto = false;

  // Line 4: while OPEN is not empty do
  size_t iterCount__ = 0;
  while ( true ) {
    iterCount__++;
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG]----------------------- search iter : " 
        << iterCount__ << " ------------------------" << std::endl;
    }

    // Line 5: P_k <- OPEN.pop (smallest key)
    NBCBSNode *node = _PopNode();
    if (!node) {
      _result.find_all_pareto = true;
      break; // terminates
    }

    // Line 6: \Pi_k <- DomPrune(C, \Pi_k)
    bool ret_flag = node->solFilter(_result.costs);

    // Line 7: if any joint path in \Pi_k is filtered then
    if (ret_flag) {
      node->clearCachedAnalysis();
      // Line 8: if \Pi_k != empty then
      if (!node->isEmptyPlans()) {
        bool flag = node->setRep();
        if (!flag) {
          std::cout << "[DEBUG] * NBMOCBS_EMOA::Search, fail to set Rep for node " << (*node) << std::endl;
          return ;
        }
        // Line 9-10: \pi_k,rep <- \Pi_k.first, g_k,rep <- g(\pi_k,rep), add P_k to OPEN with key g_k,rep
        node->key = node->rep_cvec;
        _open.insert( std::make_pair(node->key, node->id) );
      }
      // Line 11: continue
      continue;
    }

    // Line 12: cft <- DetectConflict(\pi_k,rep)
    auto cft = _DetectAndSelectConflict(node);

    // Line 13: if cft = NULL then
    if (!cft.IsValid()) {
      // Line 14: add g_k,rep to C
      _UpdateSolution(node);

      // Line 15: delete \pi_k,rep from \Pi_k
      if (!node->sorted_costIndex.empty()) {
        node->sorted_costIndex.erase(node->sorted_costIndex.begin());
      }
      node->clearCachedAnalysis();

      // Line 16: if \Pi_k != empty then
      if (!node->isEmptyPlans()) {
        bool flag = node->setRep();
        if (flag) {
          // Line 17-18: \pi_k,rep <- \Pi_k.first, g_k,rep <- g(\pi_k,rep), add P_k to OPEN with key g_k,rep
          node->key = node->rep_cvec;
          _open.insert( std::make_pair(node->key, node->id) );
        }
      }
      // Line 19: continue
      continue;
    }

    _cfts.push_back(cft);

    if ( _IfTimeout() ) {
      std::cout << "[INFO] * BBMOCBS_EMOA::Search Timeout, reach limit " << _tlimit << std::endl;
      break;
    }
    if (_result.n_expanded % (500/_nAgent) == 0) {
      std::cout << "[INFO] * BBMOCBS_EMOA::Search, after " << _result.n_expanded << " conflict splits " << std::endl;
    }

    if (getenv("H1_PLAIN")) {
      CBSConflict use = cft;
      if (getenv("H1_LASTCFT")) {   // pick the conflict of the highest-index agent pair instead
        bool done = false;
        for (int a = (int)_nAgent - 2; a >= 0 && !done; a--) {
          for (int b = (int)_nAgent - 1; b > a && !done; b--) {
            CBSConflict c2 = DetectConflict(&(node->rep_jpath.at(a)), &(node->rep_jpath.at(b)));
            if (c2.IsValid()) { c2.i = a; c2.j = b; use = c2; done = true; }
          }
        }
      }
      auto cs = use.Split();
      for (auto c : cs) { _Lsearch(c.i, node, &c); }
      _result.n_expanded++;
      continue;
    }
    // Lines 20-25: Conflict Analysis and Heuristics
    CBSConflict cft_star;
    std::vector<std::pair<int, int>> M;
    std::unordered_map<int, basic::CostVector> delta;
    std::vector<std::pair<basic::CostVector, int>> Full_k_pairs;
    std::vector<basic::CostVector> S_k;
    basic::CostVector key_prime;

    if (node->has_cached_analysis) {
      cft_star = node->cached_cft;
      M = node->cached_M;
      delta = node->cached_delta;
      Full_k_pairs = node->cached_Full_k;
      S_k = node->cached_S_k;
      key_prime = node->cached_key_prime;
    } else {
      // Line 20: (cft*, M, \Delta) <- ConflictAnalysis(P_k, \pi_k,rep, K)
      _ConflictAnalysis(node, _K, &cft_star, &M, &delta);

      // Line 21: Full_k <- { A_k + \sum over pairs (a,b) in M of \Delta_x : x chosen from {a,b} per pair }
      int m = M.size();
      int num_combinations = (m > 0) ? (1 << m) : 1;
      std::vector<basic::CostVector> full_k_vecs;
      // FIX: lower bounds must start from the node's ideal point (per objective, the sum over
      // agents of each agent's cheapest cost). g_k,rep is only exact in objective 1.
      basic::CostVector base_ideal(0L, _cdim);
      for (int x = 0; x < (int)_nAgent; x++) {
        for (size_t o = 0; o < _cdim; o++) {
          long mn = 1000000000L;
          for (const auto& itx : node->idvl_costs[x]) {
            if (itx.second[o] < mn) mn = itx.second[o];
          }
          base_ideal[o] += mn;
        }
      }
      for (int mask = 0; mask < num_combinations; mask++) {
        basic::CostVector w = getenv("H1_OLDBASE") ? node->rep_cvec : base_ideal;
        for (int p = 0; p < m; p++) {
          int choice = (mask & (1 << p)) ? M[p].second : M[p].first;
          if (!getenv("H1_NODELTA")) w = w + delta[choice];
        }
        Full_k_pairs.push_back(std::make_pair(w, mask));
        full_k_vecs.push_back(w);
      }

      // Line 22: S_k <- ND(Full_k)
      NonDomVectors(full_k_vecs, S_k);

      // Line 23: key' <- lexmax( g_k,rep , lexmin(S_k) )
      basic::CostVector lexmin_Sk = S_k.empty() ? node->rep_cvec : S_k[0];
      for (size_t s_idx = 1; s_idx < S_k.size(); s_idx++) {
        if (LexLess(S_k[s_idx], lexmin_Sk)) {
          lexmin_Sk = S_k[s_idx];
        }
      }
      key_prime = LexMax(node->rep_cvec, lexmin_Sk);

      // Cache on node
      node->has_cached_analysis = true;
      node->cached_cft = cft_star;
      node->cached_M = M;
      node->cached_delta = delta;
      node->cached_Full_k = Full_k_pairs;
      node->cached_S_k = S_k;
      node->cached_key_prime = key_prime;
    }

    // Line 24: if key' >_lex key(P_k) then add P_k to OPEN with key'; continue
    if (!getenv("H1_NOREKEY") && LexLess(node->key, key_prime)) {
      node->key = key_prime;
      _open.insert( std::make_pair(node->key, node->id) );
      continue;
    }

    // Line 25: if S_k is covered by C then discard P_k; continue
    if (!h1_nocover && IsCoveredBy(S_k, _result.costs)) {
      continue; // discard P_k
    }

    // Line 26: {w^i, w^j} <- GenerateConstraints(cft*) where cft* is a conflict of agents i and j
    auto cstrs = (getenv("H1_FIRSTCFT") ? cft : cft_star).Split();

    // Line 27: for all i' in {i, j} do
    for (auto cstr: cstrs) {
      int i_prime = cstr.i;

      // Line 28: S_l <- ND{ w in Full_k : pair (i,j) takes side i' }
      std::vector<basic::CostVector> side_vecs;
      if (!M.empty()) {
        for (const auto& item : Full_k_pairs) {
          int choice = (item.second & 1) ? M[0].second : M[0].first;
          if (choice == i_prime) {
            side_vecs.push_back(item.first);
          }
        }
      }
      std::vector<basic::CostVector> S_l;
      if (!side_vecs.empty()) {
        NonDomVectors(side_vecs, S_l);
      } else {
        S_l = S_k;
      }

      // Line 29: if S_l is covered by C then continue (skip child, no low-level search)
      if (!h1_nocover && IsCoveredBy(S_l, _result.costs)) {
        continue;
      }

      // Lines 30-38: LowLevelSearch, NonDomJointPath, key(P_l), and add to OPEN
      _LsearchWithHeuristic(i_prime, node, &cstr, S_l);
    }
    _result.n_expanded++; // # conflict resolved.
  } // end while loop

  auto tnow = std::chrono::steady_clock::now(); // for timing.
  _result.rt_search = std::chrono::duration<double>(tnow - _t0).count();  
  if (_result.n_expanded > 0) {
    _result.n_branch = _result.n_branch / _result.n_expanded; // average branching factor.
  }

  if (_result.others[1] > 0) {
    _result.others[0] = _result.others[0] / _result.others[1]; // get the averaged runtime for computeSortedPF().
  }

  _CountDupConflict();

  return ;
};

bool NBMOCBS_EMOA::_Init() {

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init start " << std::endl;
  }

  auto init_t0 = std::chrono::steady_clock::now(); // for timing.
  _result.n_expandLow = 0.0;

  // Precompute single-objective heuristic scans
  _single_dijks.resize(_nAgent);
  for (int ri = 0; ri < _nAgent; ri++) {
    _single_dijks[ri].resize(_cdim);
    for (size_t o = 0; o < _cdim; o++) {
      _single_dijks[ri][o].SetGraphPtr(&(_tempAgentGrids[ri]));
      _single_dijks[ri][o].Search(_vd[ri], o);
    }
  }

  //// create the first root node. output via a pointer to that root node.
  long nid = _GenNodeId();
  _nodes[nid].id = nid;
  NBCBSNode* out = &(_nodes[nid]); // set as output pointer
  out->idvl_paths.resize(_nAgent);
  out->idvl_costs.resize(_nAgent);

  //// plan individual pareto-optimal path for each agent.
  std::vector<CBSConstraint*> emptySetCstr;
  _result.n_initRoot = 1;
  for (int ri = 0; ri < _vo.size(); ri++) {
    _initRes[ri] = search::MOSPPResult();
    int ret_flag = _LsearchPlan( ri, emptySetCstr, &(_initRes[ri]) );
    _result.n_initRoot *= _initRes[ri].paths.size();
    // std::cout << " " << _initRes[ri].paths.size() << " ";

    if (ret_flag == 0) {
      // TODO, how to handle this exception
      std::cout << "[ERROR] * NBMOCBS_EMOA::_Init failed at robot " << ri << std::endl;
      throw std::runtime_error( "[ERROR] NBMOCBS_EMOA::_Init failed !" );
    }
    out->idvl_paths[ri] = _initRes[ri].paths;
    out->idvl_costs[ri] = _initRes[ri].costs;
  }

  auto tnow = std::chrono::steady_clock::now(); // for timing.    
  out->computeSortedPF();
  _result.others[0] += std::chrono::duration<double>(std::chrono::steady_clock::now()-tnow).count();
  _result.others[1] += 1.0;
  // no need to do solFilter, i.e., solution-filtering, at the beginning.
  bool flag = out->setRep();

  if (!flag) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init fails. Fail to set Rep for root." << _nodes[nid] << std::endl;
    return false;
  }

  // Line 2: add P_o to OPEN with key g_o,rep
  out->key = out->rep_cvec;
  out->has_cached_analysis = false;
  _open.insert( std::make_pair(out->key, out->id) );
  _result.n_generated++; // update

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init, node = " << _nodes[nid] << std::endl;
  }

  _result.n_branch = 0.0; // useless in node-based MO-CBS.
  _result.rt_initHeu = std::chrono::duration<double>(std::chrono::steady_clock::now() - init_t0).count();  

  return true;
};

NBCBSNode* NBMOCBS_EMOA::_PopNode() {
  if ( !_open.empty() ) {
    long nid = _open.begin()->second;
    NBCBSNode* out = &( _nodes[nid] );
    _open.erase(_open.begin());
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG] ++ NBMOCBS_EMOA::_PopNode node = " << (*out) << std::endl;
    }
    return out; // a node is selected.
  }

  // now _open is empty, just terminate.
  return NULL;
};


void NBMOCBS_EMOA::_UpdateSolution(NBCBSNode* n) {

  if (_result.costs.size() == 0) {
    // runtime to reach the first feasible solution.
    auto tnow = std::chrono::steady_clock::now();
    _result.rt_firstSol = std::chrono::duration<double>(tnow - _t0).count();
    _result.firstSolCost = n->rep_cvec;
  }

  // add
  if (getenv("H1_DUMPSOL")) {
    std::ofstream fo(getenv("H1_DUMPSOL"), std::ios::app);
    fo << "SOL";
    for (size_t o = 0; o < n->rep_cvec.size(); o++) fo << " " << n->rep_cvec[o];
    fo << "\n";
    for (auto& itp : n->rep_jpath) {
      fo << "A " << itp.first;
      for (auto v : itp.second) fo << " " << v;
      fo << "\n";
    }
  }
  _result.solus[_sol_id_gen] = n->rep_jpath;
  _result.costs[_sol_id_gen] = n->rep_cvec;
  _sol_id_gen++;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** NBMOCBS_EMOA::_UpdateSolution find sol node " << (*n) << std::endl;
  }
  return ;
};

bool NBMOCBS_EMOA::_GetAllCstr(int ri, NBCBSNode *in, std::vector<CBSConstraint*> *out) {
  out->clear();
  long cid = in->id;
  while (cid != -1) {
    NBCBSNode* n = &(_nodes[cid]);
    CBSConstraint* cstr_ptr = &(n->cstr);
    if (cstr_ptr->i == ri) { // a cstr imposed on agent-i.
      out->push_back( &(n->cstr) );
    }
    cid = n->parent;
  }
  return true;
};


long NBMOCBS_EMOA::_SingleObjCheapestPath(int ri, size_t obj, const std::vector<CBSConstraint*>& cstrs)
{
  basic::GridkConn& g = _tempAgentGrids[ri];
  long vo = _vo[ri];
  long vd = _vd[ri];

  std::unordered_set<std::string> node_cstrs;
  std::unordered_set<std::string> edge_cstrs;
  long last_nc_t = -1;
  long goal_nc_t = -1;
  for (auto cstr : cstrs) {
    if (!cstr->IsValid()) continue;
    if (cstr->IsNodeCstr()) {
      node_cstrs.insert(std::to_string(cstr->vi) + "@" + std::to_string(cstr->ta));
      if (cstr->vi == vd && cstr->ta > goal_nc_t) goal_nc_t = cstr->ta;
      if (cstr->ta > last_nc_t) last_nc_t = cstr->ta;
    } else if (cstr->IsEdgeCstr()) {
      edge_cstrs.insert(std::to_string(cstr->vi) + "->" + std::to_string(cstr->vj) + "@" + std::to_string(cstr->ta));
      if (cstr->ta > last_nc_t) last_nc_t = cstr->ta;
    }
  }

  // Priority queue for A* (f, g, v, t)
  typedef std::tuple<long, long, long, long> STNode;
  std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> open;

  std::unordered_map<std::string, long> best_g;

  long h0 = 0;
  if ((size_t)ri < _single_dijks.size() && obj < _single_dijks[ri].size()) {
    h0 = _single_dijks[ri][obj].GetCost(vo);
    if (h0 < 0) h0 = 0;
  }

  open.push(std::make_tuple(h0, 0L, vo, 0L));
  best_g[std::to_string(vo) + "@" + std::to_string(0)] = 0L;

  long max_t_limit = (last_nc_t > 0 ? last_nc_t : 0) + 500L;

  while (!open.empty()) {
    auto curr = open.top();
    open.pop();
    long g_val = std::get<1>(curr);
    long v = std::get<2>(curr);
    long t = std::get<3>(curr);

    std::string state_key = std::to_string(v) + "@" + std::to_string(t);
    if (best_g.find(state_key) != best_g.end() && g_val > best_g[state_key]) {
      continue;
    }

    // Goal condition: at vd and past all constraints
    if (v == vd && t > goal_nc_t) {
      return g_val;
    }

    if (t >= max_t_limit) {
      continue;
    }

    // Successor 1: Stay in place (wait action)
    std::string wait_node_key = std::to_string(v) + "@" + std::to_string(t + 1);
    if (node_cstrs.find(wait_node_key) == node_cstrs.end()) {
      long wc = (_agentCosts[ri].size() > obj) ? _agentCosts[ri][obj] : g.GetCost(v, v)[obj];
      long next_g = g_val + wc;
      long next_h = 0;
      if ((size_t)ri < _single_dijks.size() && obj < _single_dijks[ri].size()) {
        next_h = _single_dijks[ri][obj].GetCost(v);
        if (next_h < 0) next_h = 0;
      }
      long next_f = next_g + next_h;
      if (best_g.find(wait_node_key) == best_g.end() || next_g < best_g[wait_node_key]) {
        best_g[wait_node_key] = next_g;
        open.push(std::make_tuple(next_f, next_g, v, t + 1));
      }
    }

    // Successor 2: Move to neighbors
    auto succs = g.GetSuccs(v);
    for (auto u : succs) {
      std::string edge_key = std::to_string(v) + "->" + std::to_string(u) + "@" + std::to_string(t);
      if (edge_cstrs.find(edge_key) != edge_cstrs.end()) {
        continue;
      }
      std::string move_node_key = std::to_string(u) + "@" + std::to_string(t + 1);
      if (node_cstrs.find(move_node_key) != node_cstrs.end()) {
        continue;
      }
      long ec = g.GetCost(v, u)[obj];
      long next_g = g_val + ec;
      long next_h = 0;
      if ((size_t)ri < _single_dijks.size() && obj < _single_dijks[ri].size()) {
        next_h = _single_dijks[ri][obj].GetCost(u);
        if (next_h < 0) next_h = 0;
      }
      long next_f = next_g + next_h;
      if (best_g.find(move_node_key) == best_g.end() || next_g < best_g[move_node_key]) {
        best_g[move_node_key] = next_g;
        open.push(std::make_tuple(next_f, next_g, u, t + 1));
      }
    }
  }

  return 1000000000L; // 1e9 unreachable
};


void NBMOCBS_EMOA::_ConflictAnalysis(
  NBCBSNode* node, int K,
  CBSConflict* cft_star,
  std::vector<std::pair<int, int>>* M_out,
  std::unordered_map<int, basic::CostVector>* delta_out)
{
  M_out->clear();
  delta_out->clear();

  size_t D = _cdim; // number of objectives

  struct PairInfo {
    int a;
    int b;
    CBSConflict cft;
    basic::CostVector delta_a;
    basic::CostVector delta_b;
    bool is_cardinal;
    basic::CostVector score;
  };

  std::vector<PairInfo> pair_infos;
  CBSConflict first_cft;
  bool found_first_cft = false;

  // 1: for each pair of agents (a, b) whose paths in π_rep conflict do
  for (int a = 0; a < _nAgent; a++) {
    for (int b = a + 1; b < _nAgent; b++) {
      CBSConflict cft_ab = DetectConflict(&(node->rep_jpath.at(a)), &(node->rep_jpath.at(b)));
      if (!cft_ab.IsValid()) continue;
      cft_ab.i = a;
      cft_ab.j = b;

      if (!found_first_cft) {
        first_cft = cft_ab;
        found_first_cft = true;
      }

      // 3: {ω^a, ω^b} ← Split(cft_ab)
      auto cstrs = cft_ab.Split();
      CBSConstraint omega_a, omega_b;
      for (const auto& c : cstrs) {
        if (c.i == a) omega_a = c;
        if (c.i == b) omega_b = c;
      }

      // Existing constraints on a and b
      std::vector<CBSConstraint*> cstrs_a, cstrs_b;
      _GetAllCstr(a, node, &cstrs_a);
      _GetAllCstr(b, node, &cstrs_b);
      cstrs_a.push_back(&omega_a);
      cstrs_b.push_back(&omega_b);

      // 4: for x in {a, b} and each objective o = 1..M do
      // Compute Ideal_o for a and b
      basic::CostVector ideal_a(1000000000L, D);
      if (a < (int)node->idvl_costs.size()) {
        for (const auto& item : node->idvl_costs[a]) {
          for (size_t o = 0; o < D; o++) {
            if (item.second[o] < ideal_a[o]) ideal_a[o] = item.second[o];
          }
        }
      }
      basic::CostVector ideal_b(1000000000L, D);
      if (b < (int)node->idvl_costs.size()) {
        for (const auto& item : node->idvl_costs[b]) {
          for (size_t o = 0; o < D; o++) {
            if (item.second[o] < ideal_b[o]) ideal_b[o] = item.second[o];
          }
        }
      }

      basic::CostVector delta_a(0L, D);
      basic::CostVector delta_b(0L, D);

      for (size_t o = 0; o < D; o++) {
        long s_a = _SingleObjCheapestPath(a, o, cstrs_a);
        long s_b = _SingleObjCheapestPath(b, o, cstrs_b);

        if (s_a < 100000000L && ideal_a[o] < 100000000L) {
          delta_a[o] = std::max(0L, s_a - ideal_a[o]);
        } else if (s_a >= 100000000L) {
          delta_a[o] = 1000000L;
        }

        if (s_b < 100000000L && ideal_b[o] < 100000000L) {
          delta_b[o] = std::max(0L, s_b - ideal_b[o]);
        } else if (s_b >= 100000000L) {
          delta_b[o] = 1000000L;
        }
      }

      auto is_non_zero = [](const basic::CostVector& d) {
        for (size_t o = 0; o < d.size(); o++) {
          if (d[o] > 0) return true;
        }
        return false;
      };

      // 7: pair (a,b) is cardinal if Δ_a ≠ 0 and Δ_b ≠ 0
      bool is_cardinal = is_non_zero(delta_a) && is_non_zero(delta_b);

      // 8: score(a,b) ← lexmin(Δ_a, Δ_b)
      basic::CostVector score = LexMin(delta_a, delta_b);

      PairInfo pi;
      pi.a = a;
      pi.b = b;
      pi.cft = cft_ab;
      pi.delta_a = delta_a;
      pi.delta_b = delta_b;
      pi.is_cardinal = is_cardinal;
      pi.score = score;
      pair_infos.push_back(pi);
    }
  }

  // 9: cft* ← the cardinal pair with the largest score (if none, the first conflict of π_rep)
  int best_cardinal_idx = -1;
  for (size_t i = 0; i < pair_infos.size(); i++) {
    if (pair_infos[i].is_cardinal) {
      if (best_cardinal_idx == -1 || LexLess(pair_infos[best_cardinal_idx].score, pair_infos[i].score)) {
        best_cardinal_idx = i;
      }
    }
  }

  std::unordered_set<int> agents_in_M;

  if (best_cardinal_idx != -1) {
    *cft_star = pair_infos[best_cardinal_idx].cft;
    M_out->push_back(std::make_pair(cft_star->i, cft_star->j));
    agents_in_M.insert(cft_star->i);
    agents_in_M.insert(cft_star->j);
    (*delta_out)[cft_star->i] = pair_infos[best_cardinal_idx].delta_a;
    (*delta_out)[cft_star->j] = pair_infos[best_cardinal_idx].delta_b;
  } else {
    // If no cardinal pair, use first conflict of π_rep
    if (found_first_cft) {
      *cft_star = first_cft;
    } else {
      *cft_star = _DetectAndSelectConflict(node);
    }
    if (cft_star->IsValid()) {
      M_out->push_back(std::make_pair(cft_star->i, cft_star->j));
      agents_in_M.insert(cft_star->i);
      agents_in_M.insert(cft_star->j);

      bool found_delta = false;
      for (const auto& pi : pair_infos) {
        if (pi.a == cft_star->i && pi.b == cft_star->j) {
          (*delta_out)[cft_star->i] = pi.delta_a;
          (*delta_out)[cft_star->j] = pi.delta_b;
          found_delta = true;
          break;
        }
      }
      if (!found_delta) {
        (*delta_out)[cft_star->i] = basic::CostVector(0L, D);
        (*delta_out)[cft_star->j] = basic::CostVector(0L, D);
      }
    }
  }

  // 11: for the other cardinal pairs, in order of decreasing score do
  // 12:    add the pair to M if neither of its agents is already in M and |M| < K
  std::vector<PairInfo> other_cardinal;
  for (size_t i = 0; i < pair_infos.size(); i++) {
    if (pair_infos[i].is_cardinal && (int)i != best_cardinal_idx) {
      other_cardinal.push_back(pair_infos[i]);
    }
  }

  std::sort(other_cardinal.begin(), other_cardinal.end(), [](const PairInfo& p1, const PairInfo& p2) {
    return LexLess(p2.score, p1.score); // decreasing score
  });

  for (const auto& pi : other_cardinal) {
    if ((int)M_out->size() >= K) break;
    if (agents_in_M.find(pi.a) == agents_in_M.end() && agents_in_M.find(pi.b) == agents_in_M.end()) {
      M_out->push_back(std::make_pair(pi.a, pi.b));
      agents_in_M.insert(pi.a);
      agents_in_M.insert(pi.b);
      (*delta_out)[pi.a] = pi.delta_a;
      (*delta_out)[pi.b] = pi.delta_b;
    }
  }
};


int NBMOCBS_EMOA::_LsearchWithHeuristic(
  int ri, NBCBSNode* n, CBSConstraint* cp, const std::vector<basic::CostVector>& S_l)
{
  std::vector<CBSConstraint*> cstrs;
  auto retFlag = _GetAllCstr(ri, n, &cstrs);
  cstrs.push_back(cp);

  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** NBMOCBS_EMOA::_GetAllCstr" << std::endl;
    for (auto & cstr : cstrs) {
      std::cout << "[DEBUG] **** ----- cstr " << (*cstr) << std::endl;
    }
  }
  search::MOSPPResult lowRes;

  auto lowFlag = _LsearchPlan(ri, cstrs, &lowRes);
  if (!g_target.empty() && TgtOK(cstrs, ri)) {
    const std::vector<long>& tp = g_target[ri];
    basic::CostVector tc(0L, _cdim);
    for (size_t t = 0; t + 1 < tp.size(); t++) {
      if (tp[t] == tp[t+1]) tc = tc + _agentCosts[ri];
      else tc = tc + _tempAgentGrids[ri].GetCost(tp[t], tp[t+1]);
    }
    bool ok = false;
    for (const auto& itc : lowRes.costs) {
      bool le = true;
      for (size_t o = 0; o < _cdim; o++) { if (itc.second[o] > tc[o]) { le = false; break; } }
      if (le) { ok = true; break; }
    }
    if (!ok) {
      std::cerr << "*** TARGET LOST at low level: agent " << ri << " target path cost " << tc
                << " but planner returned " << lowRes.costs.size() << " vectors; cstrs:";
      for (auto c : cstrs) std::cerr << " (" << c->vi << "," << c->vj << "," << c->ta << ")";
      std::cerr << std::endl;
    }
  }
  if (getenv("H1_CHECK2")) {   // compare EMOA* with BOA* under identical constraints
    std::vector< std::vector<long> > ncs, ecs;
    for (auto c : cstrs) {
      if (c->IsNodeCstr()) { ncs.push_back(std::vector<long>{c->vi, c->ta}); }
      else if (c->IsEdgeCstr()) { ecs.push_back(std::vector<long>{c->vi, c->vj, c->ta}); }
    }
    search::MOSPPResult boaRes;
    RunBOAstGrid(_tempAgentGrids[ri], _vo[ri], _vd[ri], 60.0, _agentCosts[ri], ncs, ecs, &boaRes);
    std::set< std::vector<long> > A, B;
    for (const auto& itc : lowRes.costs) { std::vector<long> v; for (auto x : itc.second) v.push_back(x); A.insert(v); }
    for (const auto& itc : boaRes.costs) { std::vector<long> v; for (auto x : itc.second) v.push_back(x); B.insert(v); }
    if (A != B) {
      std::cerr << "*** LOWLEVEL MISMATCH agent " << ri << " emoa=" << A.size() << " boa=" << B.size() << " cstrs:";
      for (auto c : cstrs) std::cerr << " (" << c->vi << "," << c->vj << "," << c->ta << ")";
      std::cerr << std::endl;
    }
  }
  if (getenv("H1_CHECK")) {   // self-check of the single-objective bound (off unless H1_CHECK is set)
    for (size_t o = 0; o < _cdim; o++) {
      long mn = 1000000000L;
      for (const auto& itc : lowRes.costs) { if (itc.second[o] < mn) mn = itc.second[o]; }
      long sc = _SingleObjCheapestPath(ri, o, cstrs);
      if (sc > mn) {
        std::cerr << "*** BOUND VIOLATION agent " << ri << " obj " << o << ": bound search says "
                  << sc << " but the low-level planner found " << mn << std::endl;
      }
    }
  }
  if (!lowFlag || lowRes.paths.empty()) {
    return 0;
  }

  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** NBMOCBS_EMOA::_LsearchWithHeuristic, result:" << lowRes << std::endl;
  }

  long nid = _GenNodeId();
  _nodes[nid] = *n; // make a copy
  _nodes[nid].id = nid;
  _nodes[nid].parent = n->id;
  _nodes[nid].cstr = *cp; // make a copy
  _nodes[nid].clearCachedAnalysis();
  
  _nodes[nid].idvl_paths[ri].clear(); // clear old idvl paths of agent-ri.
  _nodes[nid].idvl_costs[ri].clear();

  for (const auto& ii : lowRes.paths) { // loop over all idvl paths of agent ri.
    int k = ii.first;

    std::vector<long> t2;
    std::vector<long> p2;
    UnitTimePath(lowRes.paths[k], lowRes.times[k], &p2, &t2); 
    _nodes[nid].idvl_paths[ri][k] = p2;
    _nodes[nid].idvl_costs[ri][k] = lowRes.costs[k];
  }

  auto tnow = std::chrono::steady_clock::now(); // for timing.  
  _nodes[nid].computeSortedPF();
  _result.others[0] += std::chrono::duration<double>(std::chrono::steady_clock::now()-tnow).count();
  _result.others[1] += 1.0;

  if (!_nodes[nid].isEmptyPlans()) {
    bool flag = _nodes[nid].setRep();
    if (!flag) {
      std::cout << "[DEBUG] * NBMOCBS_EMOA::Search, fail to set Rep for node " << _nodes[nid] << std::endl;
      return 0;
    }

    // Line 37: key(P_l) <- lexmax( g_l,rep , lexmin(S_l) )
    basic::CostVector key_Pl = _nodes[nid].rep_cvec;
    if (!S_l.empty()) {
      basic::CostVector lexmin_Sl = S_l[0];
      for (size_t s_idx = 1; s_idx < S_l.size(); s_idx++) {
        if (LexLess(S_l[s_idx], lexmin_Sl)) {
          lexmin_Sl = S_l[s_idx];
        }
      }
      key_Pl = LexMax(_nodes[nid].rep_cvec, lexmin_Sl);
    }
    _nodes[nid].key = key_Pl;

    // Line 38: add P_l to OPEN with key(P_l)
    _open.insert( std::make_pair(_nodes[nid].key, nid) );
    _result.n_generated++;
    _result.n_branch++;
    if (DEBUG_MOCBS) {
      std::cout << "[DEBUG] **** NBMOCBS_EMOA::_LsearchWithHeuristic, gen node:" << _nodes[nid] << std::endl;
    }
  } else {
    if (DEBUG_MOCBS) {
      std::cout << "[DEBUG] **** NBMOCBS_EMOA::_LsearchWithHeuristic, gen empty plan node:" << _nodes[nid] << std::endl;
    }
  }

  return 1;
};

int NBMOCBS_EMOA::_Lsearch(int ri, NBCBSNode* n, CBSConstraint* cp)
{
  std::vector<basic::CostVector> S_l;
  return _LsearchWithHeuristic(ri, n, cp, S_l);
};

CBSConflict NBMOCBS_EMOA::_DetectAndSelectConflict(NBCBSNode* n)
{
  return DetectConflict(&(n->rep_jpath));
};

//////////////////////////////////////////////////////////////////////////


void DetectAllConflict(std::vector<long>* p1, std::vector<long>* p2, int i, int j, std::vector<CBSConflict>* out) 
{
  // out->clear();
  if ( (p1->size() == 0) || (p2->size() == 0) ) {
    std::cout << "[WARNING] DetectConflict receive zero size path !" << std::endl;
    // throw std::runtime_error("[WARNING] DetectConflict receive zero size path !");
    return;
  }
  size_t l = p1->size() > p2->size() ? p1->size() : p2->size(); // find max.
  for (size_t idx = 0; idx < l; idx++) {
    CBSConflict cft;
    long v1 = idx < p1->size() ? p1->at(idx) : p1->back();
    long v2 = idx < p2->size() ? p2->at(idx) : p2->back();
    // std::cout << " idx = " << idx << " v1 = " << v1 << " v2 = " << v2 << std::endl;
    if ( v1 == v2 ) { // vertex conflict
      cft.i = i;
      cft.j = j;
      cft.vi = v1;
      cft.vj = v2;
      cft.ta = idx;
      cft.tb = idx;
      // break; // stop at the first conflict detected.
      out->push_back(cft) ;
    }
    size_t idy = idx + 1;
    if (idy >= l) {continue;}
    long v1b = idy < p1->size() ? p1->at(idy) : p1->back();
    long v2b = idy < p2->size() ? p2->at(idy) : p2->back();
    // std::cout << " --- v1b = " << v1b << " v2b = " << v2b << std::endl;
    if ( (v1 == v2b) && (v1b == v2) ) { // edge conflict
      cft.i = i;
      cft.j = j;
      cft.vi = v1;
      cft.vj = v2;
      cft.ta = idx;
      cft.tb = idy;
      // break; // stop at the first conflict detected.
      out->push_back(cft) ;
    }
  }
  // std::cout << " returned " << std::endl;
  return;
};


} // end namespace mapf
} // end namespace rzq
