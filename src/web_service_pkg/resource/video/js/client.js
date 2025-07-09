// WebRTC 状态日志相关的 DOM 元素
var dataChannelLog = document.getElementById('data-channel'),        // 数据通道日志元素
    iceConnectionLog = document.getElementById('ice-connection-state'), // ICE 连接状态日志元素
    iceGatheringLog = document.getElementById('ice-gathering-state'),   // ICE 收集状态日志元素
    signalingLog = document.getElementById('signaling-state');         // 信令状态日志元素

// WebRTC 核心对象
var pc = null;  // RTCPeerConnection 对象
var dc = null, dcInterval = null;  // 数据通道和其定时器

// 创建对等连接
function createPeerConnection() {
    // WebRTC 配置项
    var config = {
        sdpSemantics: 'unified-plan'  // 使用统一计划 SDP 语义
    };

    // 初始化对等连接
    pc = new RTCPeerConnection(config);

    // 监听 ICE 候选者收集状态变化
    pc.addEventListener('icegatheringstatechange', function() {
        iceGatheringLog.textContent += ' -> ' + pc.iceGatheringState;
        console.log('ICE Gathering State changed:', pc.iceGatheringState);
    }, false);
    iceGatheringLog.textContent = pc.iceGatheringState;

    // 监听 ICE 连接状态变化
    pc.addEventListener('iceconnectionstatechange', function() {
        iceConnectionLog.textContent += ' -> ' + pc.iceConnectionState;
        console.log('ICE Connection State changed:', pc.iceConnectionState);
    }, false);
    iceConnectionLog.textContent = pc.iceConnectionState;

    // 监听信令状态变化
    pc.addEventListener('signalingstatechange', function() {
        signalingLog.textContent += ' -> ' + pc.signalingState;
        console.log('Signaling State changed:', pc.signalingState);
    }, false);
    signalingLog.textContent = pc.signalingState;

    // 监听媒体轨道事件
    pc.addEventListener('track', function(evt) {
        console.log('Track event:', evt);
        if (evt.track.kind == 'video')  // 处理视频轨道
            document.getElementById('video').srcObject = evt.streams[0];
        else  // 处理音频轨道
            document.getElementById('audio').srcObject = evt.streams[0];
    });

    return pc;
}

// 协商连接过程
function negotiate() {
    // 获取选择的视频分辨率
    var videoResolutionSelect = document.getElementById('video-resolution');
    var selectedResolution = videoResolutionSelect.value;
    console.log('Selected video resolution:', selectedResolution);

    // 创建并设置本地 SDP 提议
    return pc.createOffer({offerToReceiveVideo: true}).then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        // 等待 ICE 候选者收集完成
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        // 过滤 SDP，只保留 H.264 编码器
        var localSdp = pc.localDescription.sdp;
        var filteredSdp = sdpFilterCodec('video', 'H264/90000', localSdp);
        console.log('Sending offer');
        
        // 向服务器发送提议
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: filteredSdp,
                type: pc.localDescription.type,
                video_resolution: selectedResolution
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then(function(response) {
        return response.json();  // 解析服务器响应
    }).then(function(answer) {
        return pc.setRemoteDescription(answer);  // 设置远程描述
    }).catch(function(e) {
        alert(e);  // 错误处理
    });
}

// 启动 WebRTC 连接
function start() {
    pc = createPeerConnection();
    console.log('Peer connection created');
    
    var parameters = {"ordered": true};
    dc = pc.createDataChannel('chat', parameters);
    
    // 简化数据通道处理
    dc.onmessage = function(evt) {
        if (evt.data.startsWith('pong')) {
            var elapsed_ms = current_stamp() - parseInt(evt.data.substring(5), 10);
            dc.send('latency ' + elapsed_ms);
        }
    };

    return negotiate();
}

// 停止 WebRTC 连接
function stop() {
    document.getElementById('stop').style.display = 'none';

    // 关闭数据通道
    if (dc) {
        dc.close();
        console.log('DataChannel closed');
    }

    // 停止所有收发器
    if (pc.getTransceivers) {
        pc.getTransceivers().forEach(function(transceiver) {
            if (transceiver.stop) {
                transceiver.stop();
                console.log('Transceiver stopped:', transceiver);
            }
        });
    }

    // 停止所有发送器的轨道
    pc.getSenders().forEach(function(sender) {
        sender.track.stop();
        console.log('Sender track stopped:', sender);
    });

    // 延迟关闭对等连接
    setTimeout(function() {
        pc.close();
        console.log('Peer connection closed');
    }, 500);
}

// SDP 编解码器过滤函数
function sdpFilterCodec(kind, codec, realSdp) {
    var allowed = [];  // 允许的编解码器 ID 列表
    var rtxRegex = new RegExp('a=fmtp:(\\d+) apt=(\\d+)\r$');  // RTX 格式匹配正则
    var codecRegex = new RegExp('a=rtpmap:([0-9]+) ' + escapeRegExp(codec));  // 编解码器匹配正则
    var videoRegex = new RegExp('(m=' + kind + ' .*?)( ([0-9]+))*\\s*$');  // 媒体行匹配正则
    
    var lines = realSdp.split('\n');  // 将 SDP 分割成行

    // 第一次遍历：找出所有允许的编解码器 ID
    var isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var match = lines[i].match(codecRegex);
            if (match) {
                allowed.push(parseInt(match[1]));
            }

            match = lines[i].match(rtxRegex);
            if (match && allowed.includes(parseInt(match[2]))) {
                allowed.push(parseInt(match[1]));
            }
        }
    }

    // 第二次遍历：构建过滤后的 SDP
    var skipRegex = 'a=(fmtp|rtcp-fb|rtpmap):([0-9]+)';
    var sdp = '';

    isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var skipMatch = lines[i].match(skipRegex);
            if (skipMatch && !allowed.includes(parseInt(skipMatch[2]))) {
                continue;  // 跳过不允许的编解码器
            } else if (lines[i].match(videoRegex)) {
                sdp += lines[i].replace(videoRegex, '$1 ' + allowed.join(' ')) + '\n';
            } else {
                sdp += lines[i] + '\n';
            }
        } else {
            sdp += lines[i] + '\n';
        }
    }

    console.log('Filtered SDP:', sdp);
    return sdp;
}

// 转义正则表达式特殊字符
function escapeRegExp(string) {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'); 
}