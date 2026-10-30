import React, { useEffect, useState } from 'react';
import type { McpToolDefinition, RpcLogEntry } from '../types/cad';
import {
  Check,
  Code2,
  Copy,
  GitBranch,
  Play,
  RefreshCw,
  Terminal,
  Trash2,
  UploadCloud,
} from 'lucide-react';

interface McpAndRustOpsModalProps {
  tools: McpToolDefinition[];
  rpcLogs: RpcLogEntry[];
  initialSubTab?: 'mcp_rpc' | 'rust_tests' | 'github_removal';
  onExecuteRawTool: (toolName: string, args: Record<string, unknown>) => Promise<unknown>;
  onClearLogs: () => void;
}

interface GitInfo {
  branch: string;
  lastCommit: string;
  status: string;
  remote: string;
  hasEnvToken?: boolean;
}

export const McpAndRustOpsModal: React.FC<McpAndRustOpsModalProps> = ({
  tools,
  rpcLogs,
  initialSubTab = 'mcp_rpc',
  onExecuteRawTool,
  onClearLogs,
}) => {
  const [activeSubTab, setActiveSubTab] = useState<'mcp_rpc' | 'rust_tests' | 'github_removal'>(
    initialSubTab
  );

  useEffect(() => {
    setActiveSubTab(initialSubTab);
  }, [initialSubTab]);

  // MCP Tool Tester State
  const [selectedToolName, setSelectedToolName] = useState<string>('list_parts');
  const [jsonArgsText, setJsonArgsText] = useState<string>('{}');
  const [toolResponseText, setToolResponseText] = useState<string>('');
  const [isCallingTool, setIsCallingTool] = useState(false);

  // Rust Cargo Test Runner State
  const [selectedPackage, setSelectedPackage] = useState<string>('workspace');
  const [isRunningTests, setIsRunningTests] = useState(false);
  const [testResult, setTestResult] = useState<{
    success: boolean;
    stdout: string;
    stderr: string;
    durationMs?: number;
  } | null>(null);

  // Git & Removal State
  const [gitInfo, setGitInfo] = useState<GitInfo | null>(null);
  const [gitToken, setGitToken] = useState('');
  const [gitBranch, setGitBranch] = useState('master');
  const [commitMessage, setCommitMessage] = useState('feat(web-studio): add modular MCP-CAD Web Studio');
  const [includeWebStudioInCommit, setIncludeWebStudioInCommit] = useState(false);
  const [isPushing, setIsPushing] = useState(false);
  const [pushResult, setPushResult] = useState<{ ok: boolean; message: string } | null>(null);
  const [copiedCmd, setCopiedCmd] = useState(false);

  const selectedTool = tools.find((t) => t.name === selectedToolName) || tools[0];

  useEffect(() => {
    if (selectedTool) {
      const props = (selectedTool.inputSchema as { properties?: Record<string, { type?: string }> })?.properties || {};
      const sample: Record<string, unknown> = {};
      for (const [key, val] of Object.entries(props)) {
        if (key === 'part_id') sample[key] = '<part_id>';
        else if (key === 'material') sample[key] = 'aluminum_6061_t6';
        else if (key === 'mesh_density' || key === 'quality') sample[key] = 'medium';
        else if (val?.type === 'number') sample[key] = 0.05;
        else if (val?.type === 'boolean') sample[key] = false;
        else if (val?.type === 'array') sample[key] = [];
        else sample[key] = '';
      }
      setJsonArgsText(JSON.stringify(sample, null, 2));
    }
  }, [selectedToolName, tools]);

  const fetchGitStatus = async () => {
    try {
      const res = await fetch('/api/git/info');
      const data = await res.json();
      setGitInfo(data);
      if (data.branch) setGitBranch(data.branch);
    } catch {
      // Ignore error
    }
  };

  useEffect(() => {
    if (activeSubTab === 'github_removal') {
      fetchGitStatus();
    }
  }, [activeSubTab]);

  const handleInvokeTool = async () => {
    setIsCallingTool(true);
    try {
      const parsed = JSON.parse(jsonArgsText || '{}');
      const result = await onExecuteRawTool(selectedToolName, parsed);
      setToolResponseText(JSON.stringify(result, null, 2));
    } catch (err: unknown) {
      setToolResponseText(`Error: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsCallingTool(false);
    }
  };

  const handleRunCargoTest = async () => {
    setIsRunningTests(true);
    setTestResult(null);
    try {
      const res = await fetch('/api/rust/test', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ crate: selectedPackage || 'workspace' }),
      });
      const data = await res.json();
      setTestResult(data);
    } catch (err: unknown) {
      setTestResult({
        success: false,
        stdout: '',
        stderr: err instanceof Error ? err.message : String(err),
      });
    } finally {
      setIsRunningTests(false);
    }
  };

  const handleGitPush = async () => {
    if (!gitToken.trim() && !gitInfo?.hasEnvToken) {
      setPushResult({
        ok: false,
        message:
          'Введите GitHub Personal Access Token (PAT) ниже или задайте GITHUB_TOKEN в разделе Secrets.',
      });
      return;
    }
    setIsPushing(true);
    setPushResult(null);
    try {
      const res = await fetch('/api/git/push', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          token: gitToken.trim(),
          branch: gitBranch.trim() || 'master',
          commitMessage: commitMessage.trim(),
          includeWebStudio: includeWebStudioInCommit,
        }),
      });
      const data = await res.json();
      if (data.ok || data.success) {
        setPushResult({
          ok: true,
          message: `Успешно отправлено в origin/${gitBranch}!\n${data.pushOut || data.output || ''}`,
        });
        fetchGitStatus();
      } else {
        setPushResult({
          ok: false,
          message: data.error || 'Ошибка при выполнении git push',
        });
      }
    } catch (err: unknown) {
      setPushResult({
        ok: false,
        message: err instanceof Error ? err.message : String(err),
      });
    } finally {
      setIsPushing(false);
    }
  };

  const copyRemoveCommand = () => {
    navigator.clipboard.writeText('bash scripts/remove-web-studio.sh');
    setCopiedCmd(true);
    setTimeout(() => setCopiedCmd(false), 2000);
  };

  return (
    <div className="flex-1 flex flex-col bg-[#090D16] text-slate-100 overflow-hidden">
      {/* Sub-navigation bar */}
      <div className="h-11 px-6 border-b border-slate-800/80 bg-[#0E131F] flex items-center justify-between">
        <div className="flex items-center gap-2">
          <button
            onClick={() => setActiveSubTab('mcp_rpc')}
            className={`px-3 py-1.5 rounded text-xs font-medium flex items-center gap-1.5 transition-colors cursor-pointer ${
              activeSubTab === 'mcp_rpc'
                ? 'bg-sky-500/20 text-sky-300 border border-sky-500/40'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Code2 className="w-3.5 h-3.5" />
            MCP JSON-RPC 2.0 Инспектор ({tools.length} инструментов)
          </button>
          <button
            onClick={() => setActiveSubTab('rust_tests')}
            className={`px-3 py-1.5 rounded text-xs font-medium flex items-center gap-1.5 transition-colors cursor-pointer ${
              activeSubTab === 'rust_tests'
                ? 'bg-sky-500/20 text-sky-300 border border-sky-500/40'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Terminal className="w-3.5 h-3.5" />
            Rust Cargo Test Runner
          </button>
          <button
            onClick={() => setActiveSubTab('github_removal')}
            className={`px-3 py-1.5 rounded text-xs font-medium flex items-center gap-1.5 transition-colors cursor-pointer ${
              activeSubTab === 'github_removal'
                ? 'bg-sky-500/20 text-sky-300 border border-sky-500/40'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <GitBranch className="w-3.5 h-3.5" />
            GitHub Push & Деинсталляция Web Studio
          </button>
        </div>

        <div className="text-[11px] font-mono text-slate-400">
          Rust Backend: <span className="text-emerald-400">http://127.0.0.1:8080/message</span>
        </div>
      </div>

      {/* Main Content Area */}
      <div className="flex-1 overflow-y-auto p-6">
        {activeSubTab === 'mcp_rpc' && (
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 max-w-7xl mx-auto">
            {/* Left: Interactive Tool Caller */}
            <div className="lg:col-span-5 bg-[#0E131F] border border-slate-800/90 rounded-lg p-4 flex flex-col gap-4">
              <div>
                <h3 className="text-sm font-semibold text-slate-100">
                  Прямой вызов MCP инструмента (`tools/call`)
                </h3>
                <p className="text-xs text-slate-400 mt-0.5">
                  Тестируйте все зарегистрированные инструменты `mcp-cad-server` напрямую
                </p>
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Инструмент MCP</label>
                <select
                  value={selectedToolName}
                  onChange={(e) => setSelectedToolName(e.target.value)}
                  className="w-full bg-slate-900 border border-slate-700/80 rounded px-2.5 py-2 text-xs text-slate-100 font-mono focus:outline-none focus:border-sky-500"
                >
                  {tools.map((t) => (
                    <option key={t.name} value={t.name}>
                      {t.name}
                    </option>
                  ))}
                </select>
                {selectedTool && (
                  <p className="text-[11px] text-slate-400 mt-1.5 leading-relaxed">
                    {selectedTool.description}
                  </p>
                )}
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">
                  JSON Аргументы (`arguments`)
                </label>
                <textarea
                  rows={8}
                  value={jsonArgsText}
                  onChange={(e) => setJsonArgsText(e.target.value)}
                  className="w-full bg-slate-950 border border-slate-800 rounded p-2.5 text-xs font-mono text-sky-200 focus:outline-none focus:border-sky-500"
                />
              </div>

              <button
                onClick={handleInvokeTool}
                disabled={isCallingTool}
                className="w-full py-2 px-4 rounded bg-sky-500 hover:bg-sky-400 disabled:opacity-50 text-slate-950 font-semibold text-xs flex items-center justify-center gap-2 transition-colors cursor-pointer"
              >
                <Play className="w-3.5 h-3.5" />
                {isCallingTool ? 'Выполнение JSON-RPC запроса...' : 'Отправить tools/call в Rust сервер'}
              </button>

              {toolResponseText && (
                <div>
                  <label className="block text-[11px] text-slate-400 mb-1">Ответ сервера</label>
                  <pre className="bg-slate-950 border border-slate-800 rounded p-3 text-[11px] font-mono text-emerald-300 overflow-x-auto max-h-60">
                    {toolResponseText}
                  </pre>
                </div>
              )}
            </div>

            {/* Right: Live JSON-RPC Stream Log */}
            <div className="lg:col-span-7 bg-[#0E131F] border border-slate-800/90 rounded-lg p-4 flex flex-col">
              <div className="flex items-center justify-between mb-3">
                <div>
                  <h3 className="text-sm font-semibold text-slate-100">
                    Журнал JSON-RPC 2.0 вызовов ({rpcLogs.length})
                  </h3>
                  <p className="text-xs text-slate-400">
                    Все запросы от 3D студии к бинарному Rust-серверу в реальном времени
                  </p>
                </div>
                <button
                  onClick={onClearLogs}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-xs flex items-center gap-1 cursor-pointer"
                >
                  <Trash2 className="w-3 h-3" />
                  Очистить
                </button>
              </div>

              <div className="flex-1 space-y-2.5 overflow-y-auto max-h-[540px] pr-1">
                {rpcLogs.length === 0 ? (
                  <div className="text-center py-12 text-xs text-slate-500">
                    Пока нет записей. Выполните любую CAD-операцию или сценарий.
                  </div>
                ) : (
                  rpcLogs.map((log) => (
                    <div
                      key={log.id}
                      className={`p-3 rounded border text-xs font-mono ${
                        log.status === 'error'
                          ? 'bg-rose-950/20 border-rose-800/60'
                          : 'bg-slate-900/70 border-slate-800/80'
                      }`}
                    >
                      <div className="flex items-center justify-between mb-1.5">
                        <div className="flex items-center gap-2">
                          <span
                            className={`px-1.5 py-0.5 rounded text-[10px] font-semibold ${
                              log.status === 'error'
                                ? 'bg-rose-500/20 text-rose-300'
                                : 'bg-sky-500/20 text-sky-300'
                            }`}
                          >
                            {log.toolName}
                          </span>
                          <span className="text-slate-400 text-[11px]">{log.timestamp}</span>
                        </div>
                        <span className="text-[11px] text-emerald-400 tabular-nums">
                          {log.durationMs} ms
                        </span>
                      </div>
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-2 mt-2">
                        <div>
                          <div className="text-[10px] text-slate-400 mb-0.5">Args:</div>
                          <pre className="bg-slate-950/90 p-2 rounded text-[10px] text-slate-300 overflow-x-auto max-h-28">
                            {JSON.stringify(log.args, null, 2)}
                          </pre>
                        </div>
                        <div>
                          <div className="text-[10px] text-slate-400 mb-0.5">Response:</div>
                          <pre className="bg-slate-950/90 p-2 rounded text-[10px] text-emerald-300 overflow-x-auto max-h-28">
                            {log.responseText}
                          </pre>
                        </div>
                      </div>
                    </div>
                  ))
                )}
              </div>
            </div>
          </div>
        )}

        {activeSubTab === 'rust_tests' && (
          <div className="max-w-5xl mx-auto bg-[#0E131F] border border-slate-800/90 rounded-lg p-5 space-y-4">
            <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-3 border-b border-slate-800 pb-4">
              <div>
                <h3 className="text-sm font-semibold text-slate-100">
                  Запуск модульных и интеграционных тестов Rust (`cargo test`)
                </h3>
                <p className="text-xs text-slate-400 mt-0.5">
                  Проверяет все 5 крейтов (`mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics`, `mcp-cad-viewer`, `mcp-cad-server`) на коммите `2fe786b`
                </p>
              </div>

              <div className="flex items-center gap-2">
                <select
                  value={selectedPackage}
                  onChange={(e) => setSelectedPackage(e.target.value)}
                  className="bg-slate-900 border border-slate-700 rounded px-3 py-1.5 text-xs text-slate-100 font-mono"
                >
                  <option value="workspace">Весь Workspace (--workspace)</option>
                  <option value="mcp-cad-core">mcp-cad-core (CSG, B-Rep, Gears, STL)</option>
                  <option value="mcp-cad-fea">mcp-cad-fea (TET4, Von Mises, PCG)</option>
                  <option value="mcp-cad-physics">mcp-cad-physics (Rapier3D, Gear Joints)</option>
                  <option value="mcp-cad-viewer">mcp-cad-viewer (Headless WGPU)</option>
                  <option value="mcp-cad-server">mcp-cad-server (MCP Tools)</option>
                </select>

                <button
                  onClick={handleRunCargoTest}
                  disabled={isRunningTests}
                  className="px-4 py-1.5 rounded bg-emerald-500 hover:bg-emerald-400 disabled:opacity-50 text-slate-950 font-semibold text-xs flex items-center gap-1.5 cursor-pointer"
                >
                  <Play className="w-3.5 h-3.5" />
                  {isRunningTests ? 'Выполняется cargo test...' : 'Запустить cargo test'}
                </button>
              </div>
            </div>

            {testResult ? (
              <div className="space-y-3">
                <div
                  className={`px-3 py-2 rounded border text-xs font-medium flex items-center justify-between ${
                    testResult.success
                      ? 'bg-emerald-950/30 border-emerald-700/60 text-emerald-300'
                      : 'bg-rose-950/30 border-rose-700/60 text-rose-300'
                  }`}
                >
                  <span>
                    Статус: {testResult.success ? 'Все тесты успешно пройдены (PASS)' : 'Ошибка выполнения тестов'}
                  </span>
                  {testResult.durationMs !== undefined && (
                    <span className="font-mono">{testResult.durationMs} ms</span>
                  )}
                </div>

                <div>
                  <div className="text-[11px] font-mono text-slate-400 mb-1">STDOUT:</div>
                  <pre className="bg-slate-950 border border-slate-800 rounded p-3 text-xs font-mono text-emerald-300 overflow-x-auto max-h-80">
                    {testResult.stdout || '(пусто)'}
                  </pre>
                </div>

                {testResult.stderr && (
                  <div>
                    <div className="text-[11px] font-mono text-slate-400 mb-1">STDERR (Compiler / Runner):</div>
                    <pre className="bg-slate-950 border border-slate-800 rounded p-3 text-xs font-mono text-slate-300 overflow-x-auto max-h-60">
                      {testResult.stderr}
                    </pre>
                  </div>
                )}
              </div>
            ) : (
              <div className="py-12 text-center text-xs text-slate-400">
                Нажмите «Запустить cargo test», чтобы выполнить тесты Rust-крейтов в реальном времени.
              </div>
            )}
          </div>
        )}

        {activeSubTab === 'github_removal' && (
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 max-w-7xl mx-auto">
            {/* Left: Clean Removal Design */}
            <div className="lg:col-span-6 bg-[#0E131F] border border-slate-800/90 rounded-lg p-5 space-y-4">
              <div>
                <span className="px-2 py-0.5 rounded bg-emerald-500/15 text-emerald-300 text-[11px] font-mono">
                  Zero-Lock-In Architecture
                </span>
                <h3 className="text-base font-semibold text-slate-100 mt-2">
                  Как безболезненно выпилить MCP-CAD Web Studio
                </h3>
                <p className="text-xs text-slate-300 mt-1 leading-relaxed">
                  Веб-студия спроектирована как полностью изолированный внешний слой поверх вашего Rust-сервера. В код крейтов{' '}
                  <code className="text-sky-300">crates/*</code> и <code className="text-sky-300">Cargo.toml</code> не внесено ни одной веб-зависимости.
                </p>
              </div>

              <div className="bg-slate-950 border border-slate-800 rounded p-3.5 space-y-2">
                <div className="flex items-center justify-between">
                  <span className="text-[11px] font-mono text-slate-400">
                    Удаление одной командой:
                  </span>
                  <button
                    onClick={copyRemoveCommand}
                    className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-200 text-[11px] flex items-center gap-1 cursor-pointer"
                  >
                    {copiedCmd ? <Check className="w-3 h-3 text-emerald-400" /> : <Copy className="w-3 h-3" />}
                    {copiedCmd ? 'Скопировано' : 'Копировать'}
                  </button>
                </div>
                <pre className="text-xs font-mono text-amber-300">
                  bash scripts/remove-web-studio.sh
                </pre>
              </div>

              <div className="space-y-2 text-xs text-slate-300">
                <div className="font-semibold text-slate-200">Что делает этот скрипт:</div>
                <ul className="list-disc list-inside space-y-1 text-slate-400">
                  <li>
                    Удаляет только файлы веб-оболочки (<code className="text-slate-200">src/</code>,{' '}
                    <code className="text-slate-200">server.ts</code>, <code className="text-slate-200">package.json</code>,{' '}
                    <code className="text-slate-200">vite.config.ts</code>, <code className="text-slate-200">node_modules/</code>).
                  </li>
                  <li>
                    Оставляет нетронутыми <code className="text-emerald-300">Cargo.toml</code>,{' '}
                    <code className="text-emerald-300">crates/</code>, <code className="text-emerald-300">examples/</code>,{' '}
                    <code className="text-emerald-300">docs/</code> и <code className="text-emerald-300">output/</code>.
                  </li>
                </ul>
              </div>
            </div>

            {/* Right: GitHub Push Panel */}
            <div className="lg:col-span-6 bg-[#0E131F] border border-slate-800/90 rounded-lg p-5 space-y-4">
              <div className="flex items-center justify-between">
                <div>
                  <h3 className="text-base font-semibold text-slate-100">
                    Отправка изменений в GitHub (`git push`)
                  </h3>
                  <p className="text-xs text-slate-400 mt-0.5">
                    Репозиторий: <code className="text-sky-300">hofabetlife/chordvibe-cad-mcp</code>
                  </p>
                </div>
                <button
                  onClick={fetchGitStatus}
                  className="p-1.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 cursor-pointer"
                  title="Обновить git status"
                >
                  <RefreshCw className="w-3.5 h-3.5" />
                </button>
              </div>

              {gitInfo && (
                <div className="bg-slate-950 border border-slate-800 rounded p-3 text-xs font-mono space-y-1">
                  <div>
                    <span className="text-slate-500">Удалённый репозиторий (origin):</span>{' '}
                    <span className="text-sky-300">{gitInfo.remote}</span>
                  </div>
                  <div>
                    <span className="text-slate-500">Ветка:</span>{' '}
                    <span className="text-emerald-400">{gitInfo.branch}</span>
                  </div>
                  <div>
                    <span className="text-slate-500">HEAD коммит:</span>{' '}
                    <span className="text-amber-300">{gitInfo.lastCommit}</span>
                  </div>
                </div>
              )}

              <div className="p-3 rounded bg-sky-950/30 border border-sky-800/50 text-xs text-slate-300 leading-relaxed">
                <strong className="text-sky-300">Прямой push в существующий репозиторий:</strong>{' '}
                Стандартная кнопка GitHub во внешней панели самой платформы AI Studio умеет только создавать новый репозиторий. Используйте форму ниже (или задайте секрет{' '}
                <code className="text-amber-300">GITHUB_TOKEN</code> в настройках Secrets), чтобы отправлять коммиты напрямую в существующий{' '}
                <code className="text-emerald-300">hofabetlife/chordvibe-cad-mcp</code>.
              </div>

              <div className="space-y-3">
                <div>
                  <label className="block text-[11px] text-slate-400 mb-1">
                    GitHub Personal Access Token (PAT с правами `repo` / `contents:write`)
                  </label>
                  <input
                    type="password"
                    value={gitToken}
                    onChange={(e) => setGitToken(e.target.value)}
                    placeholder={
                      gitInfo?.hasEnvToken
                        ? 'Используется GITHUB_TOKEN из AI Studio Secrets (можно оставить пустым)'
                        : 'ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'
                    }
                    className="w-full bg-slate-950 border border-slate-800 rounded px-3 py-2 text-xs font-mono text-slate-100 focus:outline-none focus:border-sky-500"
                  />
                  <p className="text-[11px] text-slate-400 mt-1">
                    {gitInfo?.hasEnvToken
                      ? 'Обнаружен секрет GITHUB_TOKEN в окружении — ввод токена вручную не требуется.'
                      : 'Токен используется однократно в памяти для прямого `git push` в hofabetlife/chordvibe-cad-mcp.'}
                  </p>
                </div>

                <div className="grid grid-cols-2 gap-3">
                  <div>
                    <label className="block text-[11px] text-slate-400 mb-1">Целевая ветка</label>
                    <input
                      type="text"
                      value={gitBranch}
                      onChange={(e) => setGitBranch(e.target.value)}
                      className="w-full bg-slate-950 border border-slate-800 rounded px-3 py-1.5 text-xs font-mono text-slate-100"
                    />
                  </div>
                  <div className="flex items-end pb-1">
                    <label className="flex items-center gap-2 text-xs text-slate-300 cursor-pointer">
                      <input
                        type="checkbox"
                        checked={includeWebStudioInCommit}
                        onChange={(e) => setIncludeWebStudioInCommit(e.target.checked)}
                        className="rounded border-slate-700"
                      />
                      Включить файлы Web Studio в коммит
                    </label>
                  </div>
                </div>

                <div>
                  <label className="block text-[11px] text-slate-400 mb-1">Сообщение коммита</label>
                  <input
                    type="text"
                    value={commitMessage}
                    onChange={(e) => setCommitMessage(e.target.value)}
                    className="w-full bg-slate-950 border border-slate-800 rounded px-3 py-1.5 text-xs font-mono text-slate-100"
                  />
                </div>

                <button
                  onClick={handleGitPush}
                  disabled={isPushing}
                  className="w-full py-2 px-4 rounded bg-sky-500 hover:bg-sky-400 disabled:opacity-50 text-slate-950 font-semibold text-xs flex items-center justify-center gap-2 cursor-pointer"
                >
                  <UploadCloud className="w-4 h-4" />
                  {isPushing ? 'Выполняется git push...' : 'Закоммитить и отправить в GitHub'}
                </button>

                {pushResult && (
                  <pre
                    className={`p-3 rounded border text-xs font-mono whitespace-pre-wrap ${
                      pushResult.ok
                        ? 'bg-emerald-950/30 border-emerald-700 text-emerald-300'
                        : 'bg-rose-950/30 border-rose-700 text-rose-300'
                    }`}
                  >
                    {pushResult.message}
                  </pre>
                )}
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};
