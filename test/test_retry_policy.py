"""連続失敗計数と再送待機の判定を検証する（ROS不要）。"""

from coverage.retry_policy import RetryPolicy


def test_initial_state_is_ready():
    policy = RetryPolicy(limit=3, backoff_sec=5.0)
    assert policy.failures == 0
    assert policy.ready(0.0)
    assert policy.state_text() == 'WAITING_NAV_RETRY 0/3'


def test_failures_schedule_backoff():
    policy = RetryPolicy(limit=3, backoff_sec=5.0)
    assert policy.record_failure(100.0) is False
    assert policy.failures == 1
    assert policy.state_text() == 'WAITING_NAV_RETRY 1/3'
    assert not policy.ready(104.9)
    assert policy.ready(105.0)


def test_give_up_at_limit():
    policy = RetryPolicy(limit=3, backoff_sec=5.0)
    assert policy.record_failure(0.0) is False
    assert policy.record_failure(5.0) is False
    assert policy.record_failure(10.0) is True
    assert policy.failures == 3


def test_limit_one_gives_up_immediately():
    policy = RetryPolicy(limit=1, backoff_sec=5.0)
    assert policy.record_failure(0.0) is True


def test_reset_clears_count_and_wait():
    policy = RetryPolicy(limit=3, backoff_sec=5.0)
    policy.record_failure(100.0)
    policy.reset()
    assert policy.failures == 0
    assert policy.ready(100.0)
    assert policy.state_text() == 'WAITING_NAV_RETRY 0/3'


def test_accept_does_not_reset_count():
    # 受理後に実行失敗が続けば上限で諦める（無限1/3ループの防止）。
    policy = RetryPolicy(limit=2, backoff_sec=5.0)
    assert policy.record_failure(0.0) is False
    # 受理相当：何も呼ばない。
    assert policy.record_failure(5.0) is True
