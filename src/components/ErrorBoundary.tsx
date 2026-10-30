import React, { Component, type ErrorInfo, type ReactNode } from 'react';
import { AlertTriangle, RefreshCw, RotateCcw } from 'lucide-react';

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

export class ErrorBoundary extends Component<Props, State> {
  public state: State = {
    hasError: false,
    error: null,
    errorInfo: null,
  };

  public static getDerivedStateFromError(error: Error): State {
    return { hasError: true, error, errorInfo: null };
  }

  public componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error('ErrorBoundary caught an unhandled error:', error, errorInfo);
    this.setState({ errorInfo });
  }

  private handleReset = () => {
    this.setState({ hasError: false, error: null, errorInfo: null });
  };

  private handleReload = () => {
    window.location.reload();
  };

  public render() {
    if (this.state.hasError) {
      return (
        <div className="h-screen w-screen flex items-center justify-center bg-[#0B0F17] text-slate-100 p-6 select-none font-sans">
          <div className="max-w-xl w-full bg-[#111827] border border-red-500/30 rounded-2xl p-6 shadow-2xl flex flex-col gap-4">
            <div className="flex items-center gap-3">
              <div className="w-10 h-10 rounded-xl bg-red-500/10 border border-red-500/30 flex items-center justify-center text-red-400 shrink-0">
                <AlertTriangle className="w-5 h-5" />
              </div>
              <div>
                <h1 className="text-lg font-semibold text-white font-display">
                  Ошибка загрузки интерфейса MCP-CAD
                </h1>
                <p className="text-xs text-slate-400">
                  Произошло непредвиденное исключение в компонентах студии
                </p>
              </div>
            </div>

            <div className="bg-[#090D16] border border-slate-800 rounded-lg p-3 text-xs font-mono text-red-300 overflow-x-auto max-h-48">
              <p className="font-semibold">{this.state.error?.name}: {this.state.error?.message}</p>
              {this.state.errorInfo?.componentStack && (
                <pre className="mt-2 text-[11px] text-slate-500 whitespace-pre-wrap">
                  {this.state.errorInfo.componentStack.trim()}
                </pre>
              )}
            </div>

            <div className="text-xs text-slate-400 bg-slate-900/60 rounded-lg p-3 border border-slate-800/80 space-y-1">
              <p className="font-medium text-slate-300">Возможные причины и рекомендации:</p>
              <ul className="list-disc pl-4 space-y-0.5 text-slate-400">
                <li>Отключено аппаратное ускорение или WebGL в браузере (проверьте <code className="text-amber-300 font-mono">chrome://gpu</code>).</li>
                <li>Блокировка cookies сторонних доменов в iframe (разрешите cookies для <code className="text-amber-300 font-mono">*.run.app</code>).</li>
                <li>Перезагрузите окно предпросмотра или страницу кнопкой ниже.</li>
              </ul>
            </div>

            <div className="flex items-center justify-end gap-2 pt-2">
              <button
                type="button"
                onClick={this.handleReset}
                className="px-4 py-2 text-xs font-medium text-slate-300 hover:text-white bg-slate-800 hover:bg-slate-700 rounded-lg border border-slate-700 flex items-center gap-1.5 transition-colors cursor-pointer"
              >
                <RotateCcw className="w-3.5 h-3.5" />
                Сбросить состояние
              </button>
              <button
                type="button"
                onClick={this.handleReload}
                className="px-4 py-2 text-xs font-semibold text-slate-950 bg-amber-500 hover:bg-amber-400 rounded-lg flex items-center gap-1.5 transition-colors cursor-pointer"
              >
                <RefreshCw className="w-3.5 h-3.5" />
                Перезагрузить страницу
              </button>
            </div>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}
