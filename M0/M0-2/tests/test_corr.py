"""tests/test_corr.py — M0-2 单元测试。

运行（在项目根或 M0-2 目录）：
    python3 -m pytest tests/ -v
依赖：pytest、numpy（均装在项目 .venv）。
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import corr  # noqa: E402

SAMPLE_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          os.pardir, "sample_data.csv")


def test_perfect_positive_correlation():
    """[1,2,3] 与 [2,4,6] 完全正相关，r 应等于 1.0。"""
    n, mx, my, r = corr.compute_correlation([1, 2, 3], [2, 4, 6])
    assert n == 3
    assert math.isclose(r, 1.0, abs_tol=1e-6)


def test_perfect_negative_correlation():
    """[1,2,3] 与 [6,4,2] 完全负相关，r 应等于 -1.0。"""
    _, _, _, r = corr.compute_correlation([1, 2, 3], [6, 4, 2])
    assert math.isclose(r, -1.0, abs_tol=1e-6)


def test_uncorrelated_series():
    """无相关序列 r 接近 0。"""
    _, _, _, r = corr.compute_correlation([1, 2, 3], [2, 2, 2.5])
    assert abs(r) < 0.5


def test_zero_variance_raises():
    """某一列取值恒定（方差为 0）时，相关系数无定义，应抛 CorrError。"""
    with pytest.raises(corr.CorrError):
        corr.compute_correlation([1, 1, 1], [1, 2, 3])


def test_too_few_samples_raises():
    """样本数不足 2 时抛 CorrError。"""
    with pytest.raises(corr.CorrError):
        corr.compute_correlation([1], [2])


def test_missing_column_raises(tmp_path):
    """csv 缺列名时抛 CorrError。"""
    p = tmp_path / "missing_col.csv"
    p.write_text("timestamp,sensor_a,note\n1,10,a\n2,20,b\n")
    with pytest.raises(corr.CorrError):
        corr.load_data(str(p), "sensor_a", "sensor_b")


def test_empty_file_raises(tmp_path):
    """空文件（仅表头或完全空）抛 CorrError。"""
    p = tmp_path / "empty.csv"
    p.write_text("")  # 完全空
    with pytest.raises(corr.CorrError):
        corr.load_data(str(p), "sensor_a", "sensor_b")


def test_nonexistent_file_raises():
    """文件不存在抛 CorrError。"""
    with pytest.raises(corr.CorrError):
        corr.load_data("/no/such/file.csv", "a", "b")


def test_sample_matches_numpy():
    """sample_data 的手写结果与 numpy.corrcoef 一致（容差 1e-6）。"""
    np = pytest.importorskip("numpy")
    xs, ys = corr.load_data(SAMPLE_CSV, "sensor_a", "sensor_b")
    _, _, _, r = corr.compute_correlation(xs, ys)
    r_np = np.corrcoef(xs, ys)[0, 1]
    assert math.isclose(r, r_np, abs_tol=1e-6)
