# =============================================================================
#  guard.sh —— 每個測試腳本開頭都要 source 它
#
#  2026-10-08：我的測試把 Oliver 正在跑的堆疊殺掉了。兩個原因：
#    ① start_empty_sitl.sh:269-271 啟動時 pkill -x px4 / pkill -f "gz sim"
#       —— 它分辨不出「我上一輪的殘留」和「別人正在跑的」
#    ② 測試腳本收尾時用很寬的 pgrep 模式 kill -KILL
#  後果不只是關掉他的視窗：兩個 drone_api_node 同時提供 /MAV1/api/*，
#  他按的起飛落到我的節點上，而且兩套 Gazebo/PX4/Nav2 搶 CPU，
#  那一輪的測量全部不可信。
#
#  「我記得要先問」已經失效過兩次，所以改成腳本層的硬性阻擋。
# =============================================================================
guard_or_die() {
    local running
    running="$(pgrep -af 'px4_sitl_default/bin|gz sim --verbose|MicroXRCEAgent|drone_api_node|controller_server|bt_navigator' \
               | grep -v shell-snapshots | grep -v 'pgrep -af')"
    if [ -n "$running" ]; then
        echo "✗ 中止：已經有東西在跑，不要殺掉它"
        echo "$running" | cut -c1-110 | sed 's/^/    /'
        echo "  確認那是殘留（而不是有人在用）之後再手動清掉。"
        return 1
    fi
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    return 0
}
