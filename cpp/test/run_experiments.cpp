// run_experiments.cpp - batch driver. Same maps, instances, cost generation and
// seeds as the original driver. Works with both public_bbmocbs and BB-MO-CBS-H1:
//   algo "bb"     -> RunBBMOCBS (mode bbmode, default 0), CSV label = name=
//   algo "mocbst" -> RunMOCBS
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "api.hpp"

struct NullBuf : public std::streambuf {
  int overflow(int c) { return c; }
};

struct MapData {
  int h = 0, w = 0;
  std::vector<std::string> rows;
};

static bool StartsWith(const std::string& s, const std::string& p) {
  return s.compare(0, p.size(), p) == 0;
}

static bool LoadMap(const std::string& path, MapData* m) {
  std::ifstream f(path.c_str());
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
    if (StartsWith(line, "height")) m->h = std::atoi(line.c_str() + 6);
    else if (StartsWith(line, "width")) m->w = std::atoi(line.c_str() + 5);
    else if (line == "map") break;
  }
  while (std::getline(f, line)) {
    if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
    if (line.empty()) continue;
    m->rows.push_back(line);
  }
  return m->h > 0 && m->w > 0 && (int)m->rows.size() >= m->h;
}

static bool IsBlocked(char ch) {
  return !(ch == '.' || ch == 'G' || ch == 'S');
}

struct Agent { int sx, sy, gx, gy; };

static bool LoadScen(const std::string& path, std::vector<Agent>* agents) {
  std::ifstream f(path.c_str());
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || StartsWith(line, "version")) continue;
    std::istringstream ss(line);
    int bucket, w, h;
    std::string mapname;
    Agent a;
    double len;
    if (ss >> bucket >> mapname >> w >> h >> a.sx >> a.sy >> a.gx >> a.gy >> len) {
      agents->push_back(a);
    }
  }
  return !agents->empty();
}

static std::map<std::string, std::string> ParseArgs(int argc, char** argv) {
  std::map<std::string, std::string> kv;
  for (int i = 1; i < argc; i++) {
    std::string a(argv[i]);
    size_t p = a.find('=');
    if (p == std::string::npos) continue;
    kv[a.substr(0, p)] = a.substr(p + 1);
  }
  return kv;
}

static std::string Get(const std::map<std::string, std::string>& kv,
                       const std::string& k, const std::string& def) {
  std::map<std::string, std::string>::const_iterator it = kv.find(k);
  return it == kv.end() ? def : it->second;
}

static std::vector<std::string> Split(const std::string& s, char d) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, d)) if (!item.empty()) out.push_back(item);
  return out;
}

static void AppendHeaderIfEmpty(const std::string& path, const std::string& header) {
  std::ifstream t(path.c_str());
  bool empty = !t || t.peek() == std::ifstream::traits_type::eof();
  t.close();
  if (empty) {
    std::ofstream o(path.c_str(), std::ios::app);
    o << header << "\n";
  }
}

// All cost vectors of the result, sorted, as "81:81;90:72;..."
static std::string CostString(const rzq::mapf::MOMAPFResult& res) {
  std::set<std::vector<long> > s;
  for (const auto& kvp : res.costs) {
    s.insert(std::vector<long>(kvp.second.begin(), kvp.second.end()));
  }
  std::string out;
  for (std::set<std::vector<long> >::const_iterator it = s.begin(); it != s.end(); ++it) {
    if (!out.empty()) out += ";";
    for (size_t k = 0; k < it->size(); k++) {
      if (k) out += ":";
      out += std::to_string((*it)[k]);
    }
  }
  return out;
}

static void RunOne(const std::string& flag, rzq::basic::GridkConn& g,
                   std::vector<long>& starts, std::vector<long>& goals,
                   rzq::mapf::MOMAPFResult* res, double tlimit, int bbmode) {
  std::vector<long> wait_cost;
  if (flag == "bb")
    rzq::mapf::RunBBMOCBS(g, starts, goals, res, tlimit, wait_cost, bbmode);
  else
    rzq::mapf::RunMOCBS(g, starts, goals, res, tlimit, wait_cost);
}

int main(int argc, char** argv) {
  std::map<std::string, std::string> kv = ParseArgs(argc, argv);
  if (!kv.count("map") || !kv.count("scen")) {
    std::cerr << "Usage: run_experiments map=<file.map> scen=<f1.scen[,f2.scen]> "
                 "[label= name= inst0= nobj= tlimit= nmin= nmax= nstep= algos=bb[,mocbst] "
                 "bbmode=0 verbose=0 out= conf= extra= dump=]" << std::endl;
    return 1;
  }

  const std::string map_path = kv["map"];
  std::string label = Get(kv, "label", "");
  if (label.empty()) {
    size_t s = map_path.find_last_of('/');
    label = map_path.substr(s == std::string::npos ? 0 : s + 1);
    size_t d = label.rfind(".map");
    if (d != std::string::npos) label = label.substr(0, d);
  }
  const std::string name = Get(kv, "name", "BB-MO-CBS");
  const int nobj = std::atoi(Get(kv, "nobj", "2").c_str());
  const double tlimit = std::atof(Get(kv, "tlimit", "300").c_str());
  const int nmin = std::atoi(Get(kv, "nmin", "2").c_str());
  const int nmax = std::atoi(Get(kv, "nmax", "20").c_str());
  const int nstep = std::atoi(Get(kv, "nstep", "2").c_str());
  const int inst0 = std::atoi(Get(kv, "inst0", "1").c_str());
  const int bbmode = std::atoi(Get(kv, "bbmode", "0").c_str());
  const bool verbose = (Get(kv, "verbose", "0") == "1");
  const std::string out_runs = Get(kv, "out", "results/runs.csv");
  const std::string out_conf = Get(kv, "conf", "results/conflicts.csv");
  const std::string out_extra = Get(kv, "extra", "results/extra.csv");
  const std::string out_dump = Get(kv, "dump", "");

  std::vector<std::pair<std::string, std::string> > todo;  // (flag, csv name)
  {
    std::vector<std::string> toks = Split(Get(kv, "algos", "bb"), ',');
    for (size_t i = 0; i < toks.size(); i++) {
      if (toks[i] == "bb") todo.push_back(std::make_pair("bb", name));
      else if (toks[i] == "mocbst") todo.push_back(std::make_pair("mocbst", "MO-CBS-t"));
      else std::cerr << "WARNING: unknown algorithm " << toks[i] << " ignored" << std::endl;
    }
  }
  if (todo.empty()) { std::cerr << "No algorithms selected." << std::endl; return 1; }

  std::vector<std::string> scen_files = Split(kv["scen"], ',');

  MapData md;
  if (!LoadMap(map_path, &md)) {
    std::cerr << "ERROR: cannot read map file " << map_path << std::endl;
    return 1;
  }
  std::cerr << "Map " << label << ": " << md.h << " rows x " << md.w << " cols, "
            << nobj << " objectives, time limit " << tlimit << "s" << std::endl;

  AppendHeaderIfEmpty(out_runs,
    "map,algo,n_agents,instance,success,num_solutions,runtime,n_obj");
  AppendHeaderIfEmpty(out_conf,
    "map,algo,n_agents,instance,all_conflicts,unique_conflicts");
  AppendHeaderIfEmpty(out_extra,
    "map,algo,n_agents,instance,n_generated,n_expanded,n_solFiltered,rt_firstSol,rt_search,n_expandLow,others");
  if (!out_dump.empty())
    AppendHeaderIfEmpty(out_dump, "map,algo,n_agents,instance,success,costs");

  NullBuf nullbuf;
  std::streambuf* cout_buf = std::cout.rdbuf();

  rzq::basic::Grid world;
  world.Resize(md.h, md.w);
  for (int y = 0; y < md.h; y++)
    for (int x = 0; x < md.w; x++)
      if (IsBlocked(md.rows[y][x])) world.Set(y, x, 1);

  for (size_t si = 0; si < scen_files.size(); si++) {
    std::vector<Agent> agents;
    if (!LoadScen(scen_files[si], &agents)) {
      std::cerr << "WARNING: cannot read scen " << scen_files[si] << ", skipped" << std::endl;
      continue;
    }
    const int instance = inst0 + (int)si;

    // Same cost generation and seed as the original run_experiments.cpp.
    std::mt19937 rng(1000003u * (unsigned)instance + 17u * (unsigned)nobj);
    std::vector<rzq::basic::Grid> costs;
    for (int k = 0; k < nobj; k++) {
      rzq::basic::Grid c;
      c.Resize(md.h, md.w, 1);
      for (int y = 0; y < md.h; y++)
        for (int x = 0; x < md.w; x++)
          c.Set(y, x, 1 + (int)(rng() % 2));
      costs.push_back(c);
    }
    rzq::basic::GridkConn g;
    g.Init(world, costs);

    for (int n = nmin; n <= nmax; n += nstep) {
      if ((size_t)n > agents.size()) break;

      std::vector<long> starts, goals;
      std::set<long> ss, gs;
      bool ok = true;
      for (int i = 0; i < n; i++) {
        long s = (long)agents[i].sy * md.w + agents[i].sx;
        long t = (long)agents[i].gy * md.w + agents[i].gx;
        if (IsBlocked(md.rows[agents[i].sy][agents[i].sx]) ||
            IsBlocked(md.rows[agents[i].gy][agents[i].gx])) ok = false;
        if (!ss.insert(s).second || !gs.insert(t).second) ok = false;
        starts.push_back(s);
        goals.push_back(t);
      }
      if (!ok) {
        std::cerr << "WARNING: instance " << instance << " N=" << n
                  << " has duplicate/blocked start or goal cells, skipped" << std::endl;
        continue;
      }

      for (size_t ai = 0; ai < todo.size(); ai++) {
        const std::string& aname = todo[ai].second;
        rzq::mapf::MOMAPFResult res;
        auto t0 = std::chrono::steady_clock::now();

        if (!verbose) std::cout.rdbuf(&nullbuf);   // hide the solver's own prints
        try {
          RunOne(todo[ai].first, g, starts, goals, &res, tlimit, bbmode);
        } catch (const std::exception& e) {
          std::cout.rdbuf(cout_buf);
          std::cerr << "ERROR in solver " << aname << ": " << e.what() << std::endl;
          continue;
        }
        std::cout.rdbuf(cout_buf);

        double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        int success = res.find_all_pareto ? 1 : 0;
        size_t nsol = res.solus.size();
        long all_c = -1, uniq_c = -1;
        if (res.others.size() >= 2) {
          all_c = (long)res.others[res.others.size() - 2];
          uniq_c = (long)res.others[res.others.size() - 1];
        }

        {
          std::ofstream o(out_runs.c_str(), std::ios::app);
          o << label << "," << aname << "," << n << "," << instance << ","
            << success << "," << nsol << "," << rt << "," << nobj << "\n";
        }
        if (all_c >= 0) {
          std::ofstream o(out_conf.c_str(), std::ios::app);
          o << label << "," << aname << "," << n << "," << instance << ","
            << all_c << "," << uniq_c << "\n";
        }
        {
          std::ofstream o(out_extra.c_str(), std::ios::app);
          o << label << "," << aname << "," << n << "," << instance << ","
            << res.n_generated << "," << res.n_expanded << "," << res.n_solFiltered << ","
            << res.rt_firstSol << "," << res.rt_search << "," << res.n_expandLow << ",";
          for (size_t k = 0; k < res.others.size(); k++) {
            if (k) o << ";";
            o << res.others[k];
          }
          o << "\n";
        }
        if (!out_dump.empty()) {
          std::ofstream o(out_dump.c_str(), std::ios::app);
          o << label << "," << aname << "," << n << "," << instance << ","
            << success << "," << CostString(res) << "\n";
        }
        std::cerr << "inst " << instance << " N=" << n << " " << aname
                  << ": " << (success ? "solved" : "TIMEOUT") << ", " << nsol
                  << " solutions, " << (all_c >= 0 ? all_c : res.n_expanded)
                  << " conflicts, " << rt << "s" << std::endl;
      }
    }
  }
  std::cerr << "Done. Results in " << out_runs << std::endl;
  return 0;
}
