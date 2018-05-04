#include "comparers/comparer.hpp"

Comparer::Comparer() {}

void Comparer::reset() {
	init_time = -1.0;
	cur_time = -1.0;

	vo_ofs.close();
	gt_ofs.close();

	vo_ofs.open("vo.txt", std::ofstream::out | std::ofstream::trunc);
	gt_ofs.open("truth.txt", std::ofstream::out | std::ofstream::trunc);

}