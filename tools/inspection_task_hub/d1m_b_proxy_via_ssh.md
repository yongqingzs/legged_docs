# D1M-B 通过本机 Clash Verge 访问外网

## 原理与已验证结果

在运行 Clash Verge 的本机主动连接 D1M-B 的 SSH，并建立反向端口转发：

```text
D1M-B:127.0.0.1:17897 --SSH 隧道--> 本机:127.0.0.1:7897 (Clash Verge)
```

板卡使用 `127.0.0.1:17897` 作为 HTTP/HTTPS 代理。转发端口只绑定板卡的回环地址，不向公网开放。2026-09-30 已从当前容器建立此隧道，并在 D1M-B 上通过代理请求 `https://www.google.com`，得到 HTTP 200。

## 操作步骤

### 1. 在本机确认 Clash Verge 代理可用

确保 Clash Verge 正在运行，`7897` 是可用的 HTTP 代理端口。在运行 Clash Verge 的本机终端执行：

```bash
curl -x http://127.0.0.1:7897 -I --max-time 15 https://www.google.com
```

看到最终的 `HTTP/2 200` 或 `HTTP/1.1 200` 即可。若在容器内建立隧道，要在**同一个容器**中运行这条命令，确认容器的 `127.0.0.1:7897` 也可访问代理。

### 2. 从本机建立 SSH 反向隧道

在能访问本机 `7897` 代理的终端执行：

```bash
ssh -N -p 20004 \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -R 127.0.0.1:17897:127.0.0.1:7897 \
  cat@47.99.202.196
```

首次连接时核对 SSH 主机指纹并输入板卡密码。本次连接观察到的 ED25519 指纹为 `SHA256:sEdagxFk+nIxpuxPQYSX+9OpQQuzGaqDLxwkf75sSng`；应以可信渠道确认的板卡指纹为准。连接成功后命令会一直占用终端且通常没有输出，这是正常现象。保持该终端、本机网络和 Clash Verge 运行；按 `Ctrl+C` 可关闭隧道。SSH 断开后，重新执行本步骤即可。

如果此前已建立后台隧道，板卡端口 `17897` 可能已被占用。代理可用时可直接跳到下一步；不要同时启动第二条使用相同端口的隧道。

### 3. 在另一个终端登录板卡并验证

```bash
ssh -p 20004 cat@47.99.202.196
```

在 **D1M-B 的终端**执行：

```bash
curl -x http://127.0.0.1:17897 -I --max-time 15 https://www.google.com
```

HTTPS 请求可能先显示 `HTTP/1.1 200 Connection established`，随后显示 Google 的 `HTTP/2 200`；以后者为准。

### 4. 让板卡上的命令使用代理

在 D1M-B 当前 shell 中执行：

```bash
export http_proxy=http://127.0.0.1:17897
export https_proxy=http://127.0.0.1:17897
curl -I --max-time 15 https://www.google.com
```

这两个变量只对当前 shell 及从中启动的程序生效。也可以只给单条命令指定代理，例如：

```bash
https_proxy=http://127.0.0.1:17897 curl -I https://www.google.com
```

不建议直接写入板卡的全局环境或系统服务配置：ROS、导航等长期运行的服务不需要通过此代理。只有识别 HTTP/HTTPS 代理变量的程序会使用它；此隧道并非全局网络路由。

## 常见问题

- 板卡提示 `Connection refused`：隧道未建立或已断开。检查本机运行隧道的终端，重新执行第 2 步。
- SSH 提示 `remote port forwarding failed for listen port 17897`：板卡上的端口可能已有一条隧道占用。先按第 3 步测试；若需要新隧道，可将第 2 至 4 步的 `17897` 一起换成其他未占用端口。
- SSH 隧道已连接，但请求超时：先按第 1 步检查本机 Clash Verge 及 `7897` 端口。本机或容器无法访问代理时，板卡也无法通过隧道访问。
- 只有部分程序能访问外网：确认程序支持 HTTP/HTTPS 代理并继承了相应环境变量；不支持代理的程序不会自动走隧道。

板卡的代理流量经 SSH 回到本机，再由本机 Clash Verge 出口访问外网。因此本机和隧道需要持续在线，板卡与 SSH 服务器之间也会产生 5G 流量。
