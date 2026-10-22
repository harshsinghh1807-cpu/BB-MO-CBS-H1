# Binary Branching Multi-Objective Conflict-Based Search with Heuristics (BB-MO-CBS-H1)

This repository provides a C++ Implementation of the Binary Branching Multi-Objective Conflict-Based Search with Heuristic Bounding Sets (BB-MO-CBS-H1) algorithm [1] for the Multi-Objective Multi-Agent Path Finding (MOMAPF) problem.

### Algorithm Highlights
- **Conflict Analysis & Cardinal Conflict Selection (Algorithm 6)**: Analyzes agent conflict pairs to compute cost increments $\Delta_x[o] = \max(0, s - \text{Ideal}_o)$ for each objective using single-objective cheapest path search under constraints. Identifies cardinal pairs and forms subset $M$ of size up to $K$.
- **Heuristic Bounding Sets ($Full_k, S_k, S_l$)**: Computes $2^{|M|}$ combinations of bounding vectors, extracts non-dominated bounds $S_k$, and prunes dominated high-level nodes using known solutions $C$.
- **Lexicographical Key Updating**: Dynamically updates search keys $key' = \text{lexmax}(g_{k,rep}, \text{lexmin}(S_k))$ and pushes back to OPEN if bounded cost increases.
- **Child Pruning (Line 29)**: Computes child side bounding set $S_l$ and skips low-level searches entirely if $S_l$ is covered by $C$.

The code is distributed for academic and non-commercial use.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Requirements

* We use CMake (3.16.3) and Make (4.2.1) on Ubuntu 20.04 to compile the code. Lower or higher version may also work. 
* We use C++11. Lower or higher version of C++ may also work.
* We massively use C++ STL, which is very common in C++ programming nowadays. We do not use any third-party libraries, and this package should be straightfoward to compile by following the instructions below.

## Project Structure

* `README.md` - This file
* `cpp/source/` - Contains the path planning source code
* `cpp/test/` - Contains example code for MO-CBS algorithms
* `cpp/include/` - Contains header files

## Instructions

* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run example in `./test_bb_mocbs `
* This example shows how to use the code.

## Notes

* For some historical reason, the classes related to BB-MO-CBS is often named with "NB" where NB stands for node-based. This name "Node-Based" is no more used and the real name should be Binary Branching (BB).

## References

Please cite the corresponding papers if you find anything in this repository helpful.

* [1] Binary Branching Multi-Objective Conflict-Based Search for Multi-Agent Path Finding\
**Zhongqiang Ren**, Jiaoyang Li, Han Zhang, Sven Koenig, Sivakumar Rathinam and Howie Choset.\
International Conference on Automated Planning and Scheduling (ICAPS), 2023.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren23bbmocbs.txt)]
[[Paper](https://wonderren.github.io/files/ren23_BBMOCBS_ICAPS23.pdf)]

* [2] A Conflict-Based Search Framework for Multi-Objective Multi-Agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE Transactions on Automation Science and Engineering (T-ASE)</i>, 2022.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21mocbs_tase.txt)]
[[Paper](https://wonderren.github.io/files/ren22_mocbs_tase_final.pdf)]
