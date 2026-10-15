# Zed 通过 SSH 连接现有 Docker 容器

本文记录将现有 Docker 容器 `jazzy_nvi` 配置为 SSH 开发环境的操作，以及在宿主机上使用 Zed 连接的方法。

## 环境与连接方式

| 项目 | 配置 |
| --- | --- |
| 容器名称 | `jazzy_nvi` |
| 容器系统 | Ubuntu 24.04 |
| Docker 网络模式 | `host` |
| 容器登录用户 | `jazzy` |
| SSH 地址 | `127.0.0.1:2222` |
| 认证方式 | SSH 公钥 |
| 容器主进程 | `bash` |

宿主机上确认网络模式的命令：

```bash
docker inspect --format '{{.HostConfig.NetworkMode}}' jazzy_nvi
```

输出为 `host`。容器共享宿主机网络，因此无需配置 Docker 端口映射，也无需重建容器。SSH 服务绑定 `127.0.0.1:2222`，供同一宿主机上的 Zed 访问。其他电脑不能直接通过宿主机的局域网地址连接这个监听地址。

连接结构如下：

```text
宿主机 Zed → SSH 127.0.0.1:2222 → 容器中的 Zed Server
                                  ├─ 项目文件
                                  ├─ 终端和运行任务
                                  └─ 语言服务器
```

## 已完成的容器端配置

以下操作已于 2026-10-14 在容器内完成，当前无需重复安装。

### 安装 SSH 服务端

```bash
sudo apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends openssh-server
```

安装时同时安装了 `openssh-sftp-server`，并升级了依赖的 `openssh-client`。容器中的 `/usr/sbin/policy-rc.d` 禁止安装阶段自动启动服务，随后通过独立配置手动启动了 SSH。

### 配置独立 SSH 服务

配置源文件保存在：

```text
/home/jazzy/scripts/container-ssh/sshd_config
```

启动脚本会将它安装为 `/etc/ssh/sshd_config_zed`，内容如下：

```text
Port 2222
ListenAddress 127.0.0.1
HostKey /etc/ssh/ssh_host_ed25519_key
PidFile /run/sshd_zed.pid

AllowUsers jazzy
PermitRootLogin no
PubkeyAuthentication yes
AuthenticationMethods publickey
AuthorizedKeysFile .ssh/authorized_keys
PasswordAuthentication no
KbdInteractiveAuthentication no
UsePAM yes

AllowTcpForwarding yes
X11Forwarding no
PrintMotd no
Subsystem sftp internal-sftp
```

当前共享网络中的 22 端口已有服务监听，因此使用 2222。连接时必须指定该端口。只有 `jazzy` 用户可以通过公钥登录；密码登录和 root 登录均已禁用。

### 启动和验证

启动脚本保存在：

```text
/home/jazzy/scripts/container-ssh/start.sh
```

在容器内执行：

```bash
bash /home/jazzy/scripts/container-ssh/start.sh
```

脚本会创建 `/run/sshd`、安装配置、生成缺失的主机密钥、验证配置并启动 `sshd`。重复执行时会检测已有服务。脚本使用的文件包括：

| 文件 | 用途 |
| --- | --- |
| `/etc/ssh/sshd_config_zed` | 实际运行配置 |
| `/run/sshd_zed.pid` | SSH 服务 PID |
| `/var/log/sshd_zed.log` | SSH 服务日志 |
| `/home/jazzy/.ssh/authorized_keys` | 允许登录的客户端公钥 |

已完成配置检查、SSH 握手和实际公钥登录验证。验证会话确认用户名为 `jazzy`，且存在 `/.dockerenv`。验证用的临时密钥及临时授权记录已删除，仍需按下文导入宿主机公钥。

## 宿主机上的连接步骤

以下命令均在宿主机终端执行，也就是运行 `docker inspect` 的终端。

### 1. 准备 SSH 密钥

如果已经有 `~/.ssh/id_ed25519.pub`，跳过生成步骤。否则执行：

```bash
ssh-keygen -t ed25519
```

使用默认保存路径，按提示设置密钥口令。不要覆盖已有密钥。如果使用其他密钥文件，后续命令需要替换对应路径。

### 2. 将宿主机公钥加入容器

```bash
cat ~/.ssh/id_ed25519.pub | docker exec -i -u jazzy jazzy_nvi sh -c \
'umask 077; mkdir -p ~/.ssh; cat >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys'
```

这里只复制公钥，私钥保留在宿主机。命令会追加公钥并设置 SSH 所需的目录和文件权限。

### 3. 测试 SSH 登录

```bash
ssh -i ~/.ssh/id_ed25519 -p 2222 jazzy@127.0.0.1
```

首次连接需要确认主机指纹。当前容器的 ED25519 主机密钥指纹为：

```text
SHA256:QewD8XrzDyn3RuVUfyfa8HJvH9Uf8VLHqHXT2aO6RCI
```

该指纹对应当前容器的主机密钥；重建容器并重新生成主机密钥后可能发生变化。

登录后执行：

```bash
whoami
test -f /.dockerenv && echo "已进入容器"
```

预期显示 `jazzy` 和“已进入容器”。执行 `exit` 返回宿主机。

### 4. 使用 Zed 连接

在 Zed 中打开 **Remote Projects → Connect New Server**，输入：

```bash
ssh -i ~/.ssh/id_ed25519 -p 2222 jazzy@127.0.0.1

ssh -p 2222 jazzy@127.0.0.1
```

连接后选择容器内的项目目录，例如 `/home/jazzy/scripts`。首次连接时，Zed 会下载并启动匹配版本的远程服务端。

如果宿主机已安装 Zed 命令行，且 SSH 能自动找到密钥，也可以直接执行：

```bash
zed ssh://jazzy@127.0.0.1:2222/home/jazzy/scripts
```

### 可选：设置 SSH 连接别名

在宿主机的 `~/.ssh/config` 中加入：

```sshconfig
Host dev-container
    HostName 127.0.0.1
    Port 2222
    User jazzy
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
```

之后可以在宿主机执行 `ssh dev-container`，并在 Zed 的新服务器连接框中使用同一命令。

## 容器重启后启动 SSH

当前没有配置 SSH 自动启动。容器重启后，在宿主机执行：

```bash
docker exec -u root jazzy_nvi bash /home/jazzy/scripts/container-ssh/start.sh
```

该命令要求容器已经处于运行状态。如果容器尚未启动，先执行：

```bash
docker start jazzy_nvi
```

容器 PID 1 是 `bash`，启动脚本直接管理 `sshd`，无需使用 `systemctl`。后续可以将脚本加入现有容器启动流程。删除并重建容器时，需要重新安装服务端，并确保配置、脚本、主机密钥和授权公钥得到保留或重新配置。

## 排查连接问题

- **Connection refused**：确认容器正在运行，并执行上面的 SSH 启动命令；连接端口应为 2222。
- **Permission denied (publickey)**：确认导入的是宿主机当前使用的公钥，登录用户是 `jazzy`，且 `.ssh` 和 `authorized_keys` 的权限分别为 700 和 600。
- **终端能连接但 Zed 无法初始化**：检查 Zed 日志和远程服务端下载网络。容器无法下载时，可以在对应 Zed SSH 连接配置中设置 `"upload_binary_over_ssh": true`，改由宿主机下载并上传。
- **项目依赖或命令找不到**：检查容器内的环境初始化。VS Code 注入的环境变量不会自动传给 Zed；ROS、虚拟环境和自定义 `PATH` 需要在相应会话中正确初始化。

在宿主机查看容器 SSH 日志：

```bash
docker exec -u root jazzy_nvi tail -n 80 /var/log/sshd_zed.log
```

## 参考

- [Zed Remote Development 官方文档](https://zed.dev/docs/remote-development)
- [Zed Dev Containers 官方文档](https://zed.dev/docs/dev-containers)

Zed 当前支持基于 `.devcontainer/devcontainer.json` 的容器开发。本文采用 SSH，以连接和继续使用现有的 `jazzy_nvi` 容器。
