# Руководство по подключению MCP-CAD к Claude Desktop

В этом руководстве описан пошаговый процесс интеграции сервера **MCP-CAD Engine** в приложение **Claude Desktop** на платформе Windows 10/11.

---

## 1. Системные требования и сборка

1. Установленный компилятор **Rust** ($\ge 1.80$):
   ```powershell
   rustc --version
   cargo --version
   ```
2. Сборка оптимизированного исполняемого файла сервера:
   ```powershell
   cd c:\Users\voley\Documents\3D_MCP
   cargo build --release -p mcp-cad-server
   ```
   После завершения сборки исполняемый файл будет доступен по пути:
   ```
   c:\Users\voley\Documents\3D_MCP\target\release\mcp-cad-server.exe
   ```

---

## 2. Конфигурация Claude Desktop

Конфигурационный файл приложения Claude Desktop на Windows расположен по стандартному пути:

```
%APPDATA%\Claude\claude_desktop_config.json
```
*(Полный путь обычно: `C:\Users\<ВашеИмя>\AppData\Roaming\Claude\claude_desktop_config.json`)*

### Шаг 1: Открытие конфигурационного файла

Вы можете быстро открыть конфигурационный файл через PowerShell:
```powershell
notepad "$env:APPDATA\Claude\claude_desktop_config.json"
```

Если файл или папка `%APPDATA%\Claude` еще не существуют, создайте их.

### Шаг 2: Добавление сервера MCP-CAD

Добавьте блок `cad-engine` в секцию `mcpServers`:

```json
{
  "mcpServers": {
    "cad-engine": {
      "command": "c:\\Users\\voley\\Documents\\3D_MCP\\target\\release\\mcp-cad-server.exe",
      "args": [
        "--mode",
        "stdio"
      ],
      "env": {
        "RUST_LOG": "info"
      }
    }
  }
}
```

> [!IMPORTANT]
> **Правило экранирования путей в Windows**: В формате JSON все обратные слэши `\` должны быть удвоены `\\`, либо заменены на прямые слэши `/`.

---

## 3. Архитектура Stdio и Логирование

Claude Desktop взаимодействует с локальными MCP-серверами через стандартные потоки ввода-вывода (`stdio`):

- **`stdin` (Standard Input)**: Сервер непрерывно считывает строки JSON-RPC от Claude.
- **`stdout` (Standard Output)**: Зарезервирован **исключительно** под ответы протокола MCP. Любой посторонний текст в `stdout` приводит к ошибке десериализации в Claude Desktop!
- **`stderr` (Standard Error)**: Сервер MCP-CAD направляет **все** информационные логи, предупреждения и трассировки Tokio/WGPU строго в `stderr`. Claude Desktop перехватывает `stderr` и сохраняет его в лог-файлы для отладки.

### Где искать логи Claude Desktop на Windows:
```
%APPDATA%\Claude\logs\
```
Файлы логов имеют вид:
- `mcp.log`
- `mcp-server-cad-engine.log`

Для просмотра логов в реальном времени используйте PowerShell:
```powershell
Get-Content -Path "$env:APPDATA\Claude\logs\mcp-server-cad-engine.log" -Wait -Tail 50
```

---

## 4. Первый запуск и проверка инструментов

1. Полностью перезапустите приложение **Claude Desktop** (закройте окно и завершите процесс в трее).
2. При открытии Claude в правом нижнем углу поля ввода появится иконка с молотком 🔨 (MCP Tools).
3. Нажмите на иконку и убедитесь, что в списке отображаются все **15 инженерных инструментов**:
   - `create_part`
   - `list_parts`
   - `delete_part`
   - `add_feature_primitive`
   - `cut_hole`
   - `modify_feature`
   - `get_feature_tree`
   - `create_joint`
   - `set_joint_motor`
   - `run_physics_simulation`
   - `run_fea_analysis`
   - `get_fea_heatmap_image`
   - `capture_viewport`
   - `export_stl`
   - `get_simulation_metrics_json`

---

## 5. Тестовый сценарий для Claude

Скопируйте и отправьте Claude следующий промпт:

> *"Claude, используй подключенный MCP-CAD Engine. Спроектируй консольную балку длиной 200 мм, высотой 40 мм и толщиной 20 мм из конструкционной стали `steel_structural_aisi1020`. Закрепи грань min_x и приложи вертикальную силу 1500 Н вниз на дальний торец. Рассчитай запас прочности (FEA) и покажи мне тепловую карту напряжений."*

### Что сделает Claude:
1. Вызовет `create_part(name="cantilever_beam", material="steel_structural_aisi1020")`.
2. Вызовет `add_feature_primitive(primitive_type="box", params={"width": 0.20, "height": 0.04, "depth": 0.02})`.
3. Вызовет `run_fea_analysis(part_id="...", fixed_faces=["min_x"], applied_forces=[{"force_vector": [0, -1500, 0]}])`.
4. Получит численный расчет (напряжение по Мизесу, FoS, прогиб).
5. Вызовет `get_fea_heatmap_image(part_id="...", camera_preset="isometric")`.
6. Отобразит в чате цветной 3D-снимок с градиентной шкалой напряжений и инженерным заключением.

---

## 6. Устранение неполадок (Troubleshooting)

### Ошибка 1: Сервер не запускается или иконка инструментов не появляется
- **Причина**: Неверный путь к `mcp-cad-server.exe` или файл еще не скомпилирован.
- **Решение**: Проверьте, что файл `c:\Users\voley\Documents\3D_MCP\target\release\mcp-cad-server.exe` существует. Запустите его вручную в консоли PowerShell:
  ```powershell
  c:\Users\voley\Documents\3D_MCP\target\release\mcp-cad-server.exe --mode stdio
  ```
  Если он выводит лог `Initializing MCP-CAD Engine...` в консоль и ожидает ввода — бинарник исправен.

### Ошибка 2: «MCP server disconnected» или JSON parse error
- **Причина**: Нарушение целостности протокола `stdio`.
- **Решение**: Убедитесь, что в коде сервера не вызывается `println!` в `stdout`. В MCP-CAD весь вывод уже направлен строго в `stderr`. Проверьте `%APPDATA%\Claude\logs\mcp-server-cad-engine.log`.

### Ошибка 3: Ошибка инициализации графического адаптера WGPU
- **Причина**: Система не нашла совместимый GPU драйвер DirectX 12 / Vulkan при запуске без физического экрана.
- **Решение**: Движок MCP-CAD автоматически выбирает доступный аппаратный или программный адаптер WGPU (WARP/Vulkan Software). Убедитесь, что в Windows установлены актуальные драйверы видеокарты.
