var pc = null;
var statsInterval = null;
var autoReconnect = true;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('audio', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                var gatherTimeout = setTimeout(resolve, 1500);
                var checkState = function() {
                    if (pc.iceGatheringState === 'complete') {
                        clearTimeout(gatherTimeout);
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        if (response.status === 503) {
            throw new Error('Server not ready, retrying...');
        }
        return response.json();
    }).then((answer) => {
        return pc.setRemoteDescription(answer);
    }).catch((e) => {
        console.error('WebRTC negotiate error:', e);
        if (autoReconnect) {
            console.log('Retrying negotiation in 3s...');
            setTimeout(() => {
                if (pc) { try { pc.close(); } catch(_) {} }
                start();
            }, 3000);
        }
    });
}

function startStats() {
    statsInterval = setInterval(() => {
        if (!pc) return;
        pc.getStats(null).then(stats => {
            let ping = '-';
            let fps = '-';
            let loss = '-';

            stats.forEach(report => {
                if (report.type === 'candidate-pair' && report.state === 'succeeded') {
                    if (report.currentRoundTripTime !== undefined) {
                        ping = (report.currentRoundTripTime * 1000).toFixed(0);
                    }
                }
                if (report.type === 'inbound-rtp' && report.kind === 'video') {
                    if (report.framesPerSecond !== undefined) {
                        fps = report.framesPerSecond.toFixed(1);
                    }
                    if (report.packetsLost !== undefined) {
                        loss = report.packetsLost;
                    }
                }
            });

            const pingEl = document.getElementById('ping');
            const fpsEl = document.getElementById('fps');
            const lossEl = document.getElementById('loss');
            if (pingEl) pingEl.innerText = ping;
            if (fpsEl) fpsEl.innerText = fps;
            if (lossEl) lossEl.innerText = loss;
        });
    }, 1000);
}

function start() {
    autoReconnect = true;
    var config = {
        sdpSemantics: 'unified-plan',
        iceServers: [{ urls: ['stun:stun.l.google.com:19302'] }]
    };

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    pc.addEventListener('connectionstatechange', () => {
        if (pc.connectionState === 'failed') {
            console.warn('WebRTC failed, reconnecting in 5s...');
            cleanupPc();
            if (autoReconnect) {
                setTimeout(() => start(), 5000);
            }
        }
    });

    document.getElementById('start').style.display = 'none';
    document.getElementById('stop').style.display = 'inline-block';
    negotiate();
    startStats();
}

function cleanupPc() {
    if (statsInterval) {
        clearInterval(statsInterval);
        statsInterval = null;
    }
    if (pc) {
        try { pc.close(); } catch(_) {}
        pc = null;
    }

    const pingEl = document.getElementById('ping');
    const fpsEl = document.getElementById('fps');
    const lossEl = document.getElementById('loss');
    if (pingEl) pingEl.innerText = '-';
    if (fpsEl) fpsEl.innerText = '-';
    if (lossEl) lossEl.innerText = '-';
}

function stop() {
    autoReconnect = false;
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';
    cleanupPc();
}
