"""Nav2ゴール再送の連続失敗計数と待機時刻をROS非依存で管理する。

``CoverageBridge`` の再試行判断（拒否・実行失敗の計数、 backoff待機、
上限到達の判定）を純粋ロジックとして切り出す。時刻は単調時計の秒
（``time.monotonic`` と同じ基準）を引数で受け取る。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RetryPolicy:
    """連続失敗を数え、再送可否と待機期限を判定する。

    拒否（ゴール不受理）と実行失敗（abort等の非成功終了）を区別せず
    連続失敗として数える。成功または新しいゾーンの受信でリセットする。
    受理時にはリセットしない。受理後に失敗が続けば上限で諦める。
    """

    limit: int = 3
    """諦めるまでの連続失敗回数。"""

    backoff_sec: float = 5.0
    """失敗後に次回再送まで待つ秒数 [s]。"""

    failures: int = 0
    """現在の連続失敗回数。"""

    next_retry_monotonic: float = 0.0
    """次回再送が可能になる単調時刻 [s]。"""

    def record_failure(self, now_monotonic: float) -> bool:
        """失敗を1件計数し、上限到達でTrue（諦め）を返す。

        上限未満なら次回再送時刻を ``backoff_sec`` 秒後に進める。
        """
        self.failures += 1
        if self.failures >= self.limit:
            return True
        self.next_retry_monotonic = now_monotonic + self.backoff_sec
        return False

    def reset(self) -> None:
        """成功時または新しいゾーンの受信時に計数と待機を破棄する。"""
        self.failures = 0
        self.next_retry_monotonic = 0.0

    def ready(self, now_monotonic: float) -> bool:
        """待機期限を過ぎて再送可能ならTrueを返す。"""
        return now_monotonic >= self.next_retry_monotonic

    def state_text(self) -> str:
        """``/coverage/state`` 通知用の再試行中テキストを返す。"""
        return f'WAITING_NAV_RETRY {self.failures}/{self.limit}'
