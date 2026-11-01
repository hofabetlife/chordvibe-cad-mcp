import React, { useMemo, useState } from 'react';
import {
  Cpu,
  Zap,
  CheckCircle2,
  ArrowRight,
  GitMerge,
  Layers,
  Sparkles,
  Terminal,
  ShieldCheck,
} from 'lucide-react';
import { CadPart, DagNode, QualityPreset } from '../types/cad';
import {
  AI_UNOPTIMIZED_SCENARIOS,
  AiUnoptimizedScenario,
  optimizeCadFeatureTree,
} from '../cad/query_optimizer';

interface CadQueryOptimizerPanelProps {
  selectedPart: CadPart | undefined;
  quality: QualityPreset;
  density: number;
  onApplyOptimizedDag: (partId: string, optimizedDag: DagNode[]) => void;
  onLoadUnoptimizedScenario: (scenario: AiUnoptimizedScenario) => void;
}

export const CadQueryOptimizerPanel: React.FC<CadQueryOptimizerPanelProps> = ({
  selectedPart,
  quality,
  density,
  onApplyOptimizedDag,
  onLoadUnoptimizedScenario,
}) => {
  const [inspectedPlanId, setInspectedPlanId] = useState<string>('');

  const report = useMemo(() => {
    if (!selectedPart) return null;
    return optimizeCadFeatureTree(
      selectedPart.id,
      selectedPart.name,
      selectedPart.dag,
      quality,
      density
    );
  }, [selectedPart, quality, density]);

  if (!selectedPart || !report) {
    return (
      <div className="p-3 text-xs text-[#94a3b8]">
        Выберите деталь для анализа плана выполнения DAG.
      </div>
    );
  }

  const activePlan =
    report.candidatePlans.find((p) => p.planId === inspectedPlanId) ||
    report.bestPlan;

  const isAlreadyOptimal =
    report.costReductionPct < 0.5 &&
    report.originalNodeCount === report.optimalNodeCount;

  return (
    <div className="space-y-3 text-xs">
      {/* Header & SQL CBO Concept Banner */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-1.5 font-semibold text-[#f1f5f9]">
            <Cpu className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>CAD Query Optimizer (CBO)</span>
          </div>
          <span className="text-[10px] font-mono text-[#38bdf8]">
            EXPLAIN ANALYZE
          </span>
        </div>
        <p className="text-[11px] text-[#94a3b8] leading-snug">
          Строит альтернативные планы вычисления Solid DAG (свертка матриц, AABB
          отсечение, закон Де Моргана для CSG, понижение 3D CSG до 2D Revolve) и
          выбирает самый дешевый план без потери топологии.
        </p>

        {/* Preset Switcher for Unoptimized AI Scenarios */}
        <div className="pt-1.5 border-t border-[#242936] space-y-1.5">
          <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b] block">
            Тестовые цепочки ИИ (Неоптимальные DAG):
          </span>
          <div className="space-y-1">
            {AI_UNOPTIMIZED_SCENARIOS.map((scen) => (
              <button
                key={scen.id}
                onClick={() => {
                  onLoadUnoptimizedScenario(scen);
                  setInspectedPlanId('');
                }}
                className="w-full text-left px-2 py-1.5 rounded bg-[#11141b] hover:bg-[#1b212d] border border-[#262d3d] transition-colors cursor-pointer flex items-center justify-between gap-2"
              >
                <div className="min-w-0">
                  <div className="text-[11px] font-medium text-[#e2e8f0] truncate">
                    {scen.title.split(' (')[0]}
                  </div>
                  <div className="text-[10px] font-mono text-[#64748b] truncate">
                    {scen.dag.length} ops · {scen.shortTag}
                  </div>
                </div>
                <span className="text-[10px] font-mono text-[#38bdf8] shrink-0">
                  Загрузить →
                </span>
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* Cost & Speedup Telemetry Comparison */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2.5">
        <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
          <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
            Оценка стоимости (Cost Model)
          </span>
          <span
            className={`text-[10px] font-mono font-semibold ${
              isAlreadyOptimal ? 'text-[#10b981]' : 'text-[#f59e0b]'
            }`}
          >
            {isAlreadyOptimal
              ? 'OPTIMAL PLAN ACTIVE'
              : `ЭКОНОМИЯ -${report.costReductionPct}%`}
          </span>
        </div>

        <div className="grid grid-cols-3 gap-1.5 text-center font-mono">
          <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
            <span className="text-[9px] text-[#64748b] block">Cost (План)</span>
            <div className="text-[11px] font-semibold flex items-center justify-center gap-1 mt-0.5">
              <span className="text-[#94a3b8] line-through">
                {report.originalCost}
              </span>
              <ArrowRight className="w-2.5 h-2.5 text-[#38bdf8]" />
              <span className="text-[#10b981]">{report.optimalCost}</span>
            </div>
          </div>

          <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
            <span className="text-[9px] text-[#64748b] block">Узлы DAG</span>
            <div className="text-[11px] font-semibold flex items-center justify-center gap-1 mt-0.5">
              <span className="text-[#94a3b8]">{report.originalNodeCount}</span>
              <ArrowRight className="w-2.5 h-2.5 text-[#38bdf8]" />
              <span className="text-[#38bdf8]">{report.optimalNodeCount}</span>
            </div>
          </div>

          <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
            <span className="text-[9px] text-[#64748b] block">Sliver Risk</span>
            <div className="text-[11px] font-semibold flex items-center justify-center gap-1 mt-0.5">
              <span className="text-[#f87171]">
                {report.candidatePlans[0].costBreakdown.sliverRiskScore}%
              </span>
              <ArrowRight className="w-2.5 h-2.5 text-[#38bdf8]" />
              <span className="text-[#10b981]">
                {report.bestPlan.costBreakdown.sliverRiskScore}%
              </span>
            </div>
          </div>
        </div>

        {/* Geometric & Topological Equivalence Invariant Proof */}
        <div className="bg-[#12151c] border border-[#222734] rounded p-2 space-y-1 text-[10px] font-mono">
          <div className="flex items-center justify-between text-[#cbd5e1]">
            <span className="flex items-center gap-1">
              <ShieldCheck className="w-3.5 h-3.5 text-[#10b981]" />
              <span>Инварианты топологии:</span>
            </span>
            <span className="text-[#10b981] font-semibold">
              {report.invariantCheck.equivalenceVerified
                ? 'ЭКВИВАЛЕНТНО (PASS)'
                : 'CHECK DELTA'}
            </span>
          </div>
          <div className="flex justify-between text-[#94a3b8]">
            <span>Объем V (см³):</span>
            <span className="text-[#e2e8f0]">
              {(report.invariantCheck.volumeBeforeM3 * 1e6).toFixed(2)} →{' '}
              {(report.invariantCheck.volumeAfterM3 * 1e6).toFixed(2)} (Δ{' '}
              {report.invariantCheck.volumeDeltaPct}%)
            </span>
          </div>
          <div className="flex justify-between text-[#94a3b8]">
            <span>Euler Genus (g):</span>
            <span className="text-[#10b981]">
              g = {report.invariantCheck.genusBefore} → g ={' '}
              {report.invariantCheck.genusAfter} (Watertight)
            </span>
          </div>
        </div>

        {/* Apply Optimal Plan CTA */}
        <button
          onClick={() =>
            onApplyOptimizedDag(selectedPart.id, report.bestPlan.dag)
          }
          disabled={isAlreadyOptimal}
          className={`w-full py-2 rounded font-semibold text-xs flex items-center justify-center gap-1.5 transition-colors cursor-pointer ${
            isAlreadyOptimal
              ? 'bg-[#1e2533] text-[#64748b] border border-[#283042] cursor-default'
              : 'bg-[#10b981] hover:bg-[#059669] text-[#0c0e12]'
          }`}
        >
          <Zap className="w-3.5 h-3.5 fill-current" />
          <span>
            {isAlreadyOptimal
              ? 'Дерево DAG уже оптимизировано'
              : `Применить лучший план (${report.originalNodeCount} → ${report.optimalNodeCount} оп., ×${report.speedupFactor} быстрее)`}
          </span>
        </button>
      </div>

      {/* Candidate Plans Selector (SQL Optimizer Alternatives) */}
      <div className="space-y-1.5">
        <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
          <span>Альтернативные планы ({report.candidatePlans.length})</span>
          <Layers className="w-3 h-3 text-[#38bdf8]" />
        </div>

        <div className="space-y-1.5">
          {report.candidatePlans.map((plan) => {
            const isInspected = activePlan.planId === plan.planId;
            const costRatio = Math.min(
              100,
              Math.max(
                10,
                Math.round(
                  (plan.costBreakdown.totalCostScore /
                    Math.max(1, report.originalCost)) *
                    100
                )
              )
            );
            return (
              <button
                key={plan.planId}
                onClick={() => setInspectedPlanId(plan.planId)}
                className={`w-full text-left p-2 rounded border transition-colors cursor-pointer space-y-1.5 ${
                  isInspected
                    ? 'bg-[#1a2130] border-[#38bdf8]'
                    : 'bg-[#181c24] border-[#262b36] hover:bg-[#1b202b]'
                }`}
              >
                <div className="flex items-center justify-between gap-1">
                  <span className="font-semibold text-[11px] text-[#f1f5f9] truncate">
                    {plan.strategyName}
                  </span>
                  <span
                    className={`text-[10px] font-mono shrink-0 ${
                      plan.isSelectedBest
                        ? 'text-[#10b981] font-semibold'
                        : 'text-[#94a3b8]'
                    }`}
                  >
                    {plan.isSelectedBest ? '★ BEST ' : ''}Cost:{' '}
                    {plan.costBreakdown.totalCostScore}
                  </span>
                </div>

                <div className="text-[10px] font-mono text-[#94a3b8] truncate">
                  SQL: {plan.sqlEquivalent}
                </div>

                {/* Relative Cost Progress Bar */}
                <div className="w-full h-1.5 bg-[#0f1218] rounded overflow-hidden">
                  <div
                    className={`h-full transition-all ${
                      plan.isSelectedBest ? 'bg-[#10b981]' : 'bg-[#f59e0b]'
                    }`}
                    style={{ width: `${costRatio}%` }}
                  />
                </div>

                <div className="flex items-center justify-between text-[10px] font-mono text-[#64748b]">
                  <span>{plan.dag.length} узлов DAG</span>
                  <span>·</span>
                  <span>~{plan.costBreakdown.estimatedTimeMs} мс</span>
                  <span>·</span>
                  <span>BSP глубина: {plan.costBreakdown.bspSplitDepth}</span>
                </div>
              </button>
            );
          })}
        </div>
      </div>

      {/* Applied Algebraic Rewrite Rules */}
      {activePlan.appliedRules.length > 0 && (
        <div className="space-y-1.5">
          <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
            <span>
              Примененные правила ({activePlan.appliedRules.length})
            </span>
            <GitMerge className="w-3 h-3 text-[#10b981]" />
          </div>

          <div className="space-y-1.5">
            {activePlan.appliedRules.map((rule, idx) => (
              <div
                key={`${rule.ruleId}_${idx}`}
                className="bg-[#181c24] border border-[#282e3d] rounded p-2 space-y-1"
              >
                <div className="flex items-center justify-between gap-1">
                  <span className="text-[11px] font-semibold text-[#38bdf8]">
                    {rule.title}
                  </span>
                  <span className="text-[10px] font-mono text-[#10b981] shrink-0">
                    -{rule.costSaved} cost
                  </span>
                </div>
                <div className="text-[10px] font-mono text-[#f59e0b]">
                  Аналог SQL: {rule.sqlAnalogy}
                </div>
                <p className="text-[10px] text-[#94a3b8] leading-snug">
                  {rule.description}
                </p>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* PostgreSQL-Style EXPLAIN (ANALYZE, COSTS) Execution Plan Tree */}
      <div className="space-y-1.5">
        <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
          <span>EXPLAIN (ANALYZE, COSTS) Tree</span>
          <Terminal className="w-3 h-3 text-[#f59e0b]" />
        </div>
        <pre className="bg-[#0f1218] border border-[#242936] rounded p-2 text-[9.5px] font-mono text-[#cbd5e1] overflow-x-auto leading-relaxed">
          {activePlan.explainTree.join('\n')}
        </pre>
      </div>
    </div>
  );
};
