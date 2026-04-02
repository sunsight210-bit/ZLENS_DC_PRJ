# settings.local.json 权限规则说明

JSON 不支持注释，本文件作为 `settings.local.json` 的配套说明。

## allow — 免授权操作

### 内置工具（全局放开）
| 规则 | 说明 |
|------|------|
| `Read` | 读取任意文件 |
| `Glob` | 按模式搜索文件名 |
| `Grep` | 按内容搜索文件 |
| `MultiEdit` / `Edit` / `Write` | 编辑/写入文件（有 diff 预览） |
| `WebSearch` / `WebFetch` | 网络搜索与网页抓取 |
| `Skill` | 调用所有技能 |

### 只读 Shell 命令
| 规则 | 说明 |
|------|------|
| `git` | 版本控制 |
| `ls` / `find` / `grep` / `wc` | 文件浏览与统计 |
| `nm` / `xxd` | 二进制分析（符号表、十六进制查看） |
| `mkdir` / `cd` | 创建目录、切换目录 |

### 构建工具链
| 规则 | 说明 |
|------|------|
| `make` / `cmake` / `ninja` / `ctest` | 编译与测试 |

### Python 与包管理
| 规则 | 说明 |
|------|------|
| `python3` / `python` / `pip3` | Python 执行与包管理 |
| `which` / `uvx` | 查找命令路径、uv 工具 |
| `dpkg` / `snap` / `flatpak` | 系统包查询 |

### GitHub CLI
| 规则 | 说明 |
|------|------|
| `gh` | GitHub CLI 所有子命令（PR、issue 等） |

### STM32 工具链（烧录、调试）
| 规则 | 说明 |
|------|------|
| `bash scripts/` | 项目 scripts/ 下所有脚本（flash_and_test.sh 等） |
| `/home/cbn/st/` | ST 工具链安装目录下所有可执行文件 |
| `OPENOCD=` / `SCRIPTS=` / `PROGRAMMER=` / `ELF=` / `SWO_LOG=` | 环境变量赋值（设置工具链路径） |
| `$OPENOCD` / `$PROGRAMMER` | 通过变量调用 OpenOCD / STM32 Programmer |
| `STM32_Programmer_CLI` / `openocd` | 直接调用烧录/调试工具 |
| `pkill` | 终止残留的 OpenOCD/Programmer 进程 |

### build 目录操作
| 规则 | 说明 |
|------|------|
| `build/` | 执行 build 目录下的测试二进制 |
| `rm -rf build` / `rm -r build` / `rm -f build/` / `rm build/` | 允许清理 build 目录 |

### 其他
| 规则 | 说明 |
|------|------|
| `libreoffice` | 转换文档格式（docx → txt） |
| `for` / `do` / `done` / `xargs` / `__NEW_LINE_` | Shell 循环与多行命令支持 |

---

## deny — 禁止操作（即使在 allow 中也会被拦截）

| 规则 | 说明 |
|------|------|
| `rm -rf` / `rm -r` / `rm` | 禁止删除文件（build 目录除外，已在 allow 中放行） |
| `sudo` | 禁止提权 |
| `dd` / `mkfs` | 禁止磁盘级操作 |
| `chmod 777` | 禁止开放所有权限 |
| `sed -i` | 禁止通过 Bash 原地编辑文件（应走 Edit 工具） |
| `truncate` | 禁止清空文件 |
| `> ` | 禁止重定向覆盖文件 |
| `mv` | 禁止移动/重命名文件 |
