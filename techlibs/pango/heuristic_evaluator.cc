/*
 * heuristic_evaluator.cc
 *
 * HeuristicEvaluator 实现
 *
 * 作者：根据8号文档Section 3.5实现
 * 日期：2025-10-04
 */

#include "heuristic_evaluator.h"

YOSYS_NAMESPACE_BEGIN

HeuristicEvaluator::HeuristicEvaluator(
	Module *module,
	MappingContext *context,
	TimingAnalyzer *timing)
	: module(module), context(context), timing(timing), current_mode(EvaluationMode::DEPTH)
{
}

int HeuristicEvaluator::computeDepth(const SingleCut &cut)
{
	return timing->getCutDepth(cut.inputs);
}

double HeuristicEvaluator::computeAreaFlow(const SingleCut &cut)
{
	int area = 0;
	for (SigBit input : cut.inputs) {
		area += context->getExactArea(input);
	}

	int refs = std::max(1, context->getFanoutRefs(cut.output));
	return (double)(area + 1) / refs;
}

int HeuristicEvaluator::computeArea(const SingleCut &cut)
{
	return context->getExactArea(cut.output);
}

bool HeuristicEvaluator::compareSingleCuts(const SingleCut &a, const SingleCut &b)
{
	switch (current_mode) {
		case EvaluationMode::DEPTH:
			return compareByDepth(a, b);

		case EvaluationMode::AREA_FLOW:
			return compareByAreaFlow(a, b);

		case EvaluationMode::EXACT_AREA:
			return compareByExactArea(a, b);

		default:
			return false;
	}
}

bool HeuristicEvaluator::compareByDepth(const SingleCut &a, const SingleCut &b)
{
	int depth_a = computeDepth(a);
	int depth_b = computeDepth(b);

	if (depth_a != depth_b) {
		return depth_a < depth_b;
	}

	// 深度相同，比较面积流
	return computeAreaFlow(a) < computeAreaFlow(b);
}

bool HeuristicEvaluator::compareByAreaFlow(const SingleCut &a, const SingleCut &b)
{
	double af_a = computeAreaFlow(a);
	double af_b = computeAreaFlow(b);

	if (std::abs(af_a - af_b) > 1e-6) {
		return af_a < af_b;
	}

	// 面积流相同，比较深度
	return computeDepth(a) < computeDepth(b);
}

bool HeuristicEvaluator::compareByExactArea(const SingleCut &a, const SingleCut &b)
{
	int area_a = computeArea(a);
	int area_b = computeArea(b);

	if (area_a != area_b) {
		return area_a < area_b;
	}

	// 面积相同，比较深度
	return computeDepth(a) < computeDepth(b);
}

YOSYS_NAMESPACE_END
