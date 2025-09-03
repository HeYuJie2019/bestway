#!/usr/bin/env python3
"""
温度数据共享存储模块
用于在不同节点之间共享温度数据
"""

import threading
import json
import time
from typing import List, Dict, Any, Optional
import os

class TemperatureDataStorage:
    """温度数据存储类，提供线程安全的温度数据读写"""
    
    def __init__(self, storage_file: str = "/tmp/temperature_data.json"):
        """
        初始化温度数据存储
        
        Args:
            storage_file: 存储文件路径
        """
        self.storage_file = storage_file
        self._lock = threading.Lock()
        self._temperature_matrix = []
        self._max_temp = 0.0
        self._min_temp = 0.0
        self._last_update_time = 0.0
        self._statistics = {
            'total_frames': 0,
            'avg_max_temp': 0.0,
            'avg_min_temp': 0.0,
            'update_rate': 0.0
        }
        
        # 确保存储目录存在
        os.makedirs(os.path.dirname(storage_file), exist_ok=True)
        
        # 尝试加载已有数据
        self._load_from_file()
    
    def update_temperature_data(self, matrix, rows: int, cols: int) -> None:
        """
        更新温度矩阵数据
        
        Args:
            matrix: 温度数据矩阵（可以是列表或array.array）
            rows: 矩阵行数
            cols: 矩阵列数
        """
        with self._lock:
            current_time = time.time()
            
            # 将matrix转换为列表（兼容array.array和list）
            if hasattr(matrix, 'tolist'):
                matrix_list = matrix.tolist()
            elif hasattr(matrix, '__iter__'):
                matrix_list = list(matrix)
            else:
                matrix_list = []
            
            # 计算最大最小温度
            if matrix_list:
                max_temp = max(matrix_list)
                min_temp = min(matrix_list)
            else:
                max_temp = 0.0
                min_temp = 0.0
            
            # 更新数据
            self._temperature_matrix = matrix_list
            self._max_temp = max_temp
            self._min_temp = min_temp
            self._last_update_time = current_time
            
            # 更新统计信息
            self._statistics['total_frames'] += 1
            if self._statistics['total_frames'] == 1:
                self._statistics['avg_max_temp'] = max_temp
                self._statistics['avg_min_temp'] = min_temp
            else:
                # 计算移动平均
                alpha = 0.1  # 平滑因子
                self._statistics['avg_max_temp'] = (1 - alpha) * self._statistics['avg_max_temp'] + alpha * max_temp
                self._statistics['avg_min_temp'] = (1 - alpha) * self._statistics['avg_min_temp'] + alpha * min_temp
            
            # 计算更新率
            if self._statistics['total_frames'] > 1:
                time_diff = current_time - (self._last_update_time - 1)  # 简化计算
                if time_diff > 0:
                    self._statistics['update_rate'] = 1.0 / time_diff
            
            # 保存到文件
            self._save_to_file(rows, cols)
    
    def get_temperature_data(self) -> Dict[str, Any]:
        """
        获取当前温度数据
        
        Returns:
            包含温度数据和统计信息的字典
        """
        with self._lock:
            return {
                'matrix': self._temperature_matrix.copy(),
                'max_temp': self._max_temp,
                'min_temp': self._min_temp,
                'last_update_time': self._last_update_time,
                'statistics': self._statistics.copy(),
                'data_age': time.time() - self._last_update_time if self._last_update_time > 0 else float('inf')
            }
    
    def get_max_temperature(self) -> float:
        """获取最大温度"""
        with self._lock:
            return self._max_temp
    
    def get_min_temperature(self) -> float:
        """获取最小温度"""
        with self._lock:
            return self._min_temp
    
    def get_temperature_range(self) -> tuple:
        """获取温度范围 (min, max)"""
        with self._lock:
            return (self._min_temp, self._max_temp)
    
    def is_data_fresh(self, max_age_seconds: float = 5.0) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_age_seconds: 最大数据年龄（秒）
            
        Returns:
            数据是否在指定时间内更新过
        """
        with self._lock:
            if self._last_update_time == 0:
                return False
            return (time.time() - self._last_update_time) <= max_age_seconds
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._lock:
            stats = self._statistics.copy()
            stats['data_age'] = time.time() - self._last_update_time if self._last_update_time > 0 else float('inf')
            return stats
    
    def _save_to_file(self, rows: int, cols: int) -> None:
        """保存数据到文件"""
        try:
            data = {
                'temperature_matrix': self._temperature_matrix,
                'max_temp': self._max_temp,
                'min_temp': self._min_temp,
                'last_update_time': self._last_update_time,
                'matrix_shape': {'rows': rows, 'cols': cols},
                'statistics': self._statistics,
                'timestamp': time.time()
            }
            
            with open(self.storage_file, 'w') as f:
                json.dump(data, f, indent=2)
                
        except Exception as e:
            print(f"保存温度数据到文件失败: {e}")
    
    def _load_from_file(self) -> None:
        """从文件加载数据"""
        try:
            if os.path.exists(self.storage_file):
                with open(self.storage_file, 'r') as f:
                    data = json.load(f)
                
                # 检查数据年龄，只加载新鲜数据
                if 'timestamp' in data:
                    age = time.time() - data['timestamp']
                    if age < 300:  # 5分钟内的数据
                        self._temperature_matrix = data.get('temperature_matrix', [])
                        self._max_temp = data.get('max_temp', 0.0)
                        self._min_temp = data.get('min_temp', 0.0)
                        self._last_update_time = data.get('last_update_time', 0.0)
                        self._statistics = data.get('statistics', {
                            'total_frames': 0,
                            'avg_max_temp': 0.0,
                            'avg_min_temp': 0.0,
                            'update_rate': 0.0
                        })
                        
        except Exception as e:
            print(f"从文件加载温度数据失败: {e}")

# 全局单例实例
_global_temperature_storage = None
_storage_lock = threading.Lock()

def get_temperature_storage(storage_file: str = "/tmp/temperature_data.json") -> TemperatureDataStorage:
    """
    获取全局温度数据存储实例（单例模式）
    
    Args:
        storage_file: 存储文件路径
        
    Returns:
        TemperatureDataStorage实例
    """
    global _global_temperature_storage
    
    with _storage_lock:
        if _global_temperature_storage is None:
            _global_temperature_storage = TemperatureDataStorage(storage_file)
        return _global_temperature_storage

# 便民函数
def update_temperature(matrix: List[float], rows: int, cols: int) -> None:
    """便民函数：更新温度数据"""
    storage = get_temperature_storage()
    storage.update_temperature_data(matrix, rows, cols)

def get_current_temperature() -> Dict[str, Any]:
    """便民函数：获取当前温度数据"""
    storage = get_temperature_storage()
    return storage.get_temperature_data()

def get_temperature_range() -> tuple:
    """便民函数：获取温度范围"""
    storage = get_temperature_storage()
    return storage.get_temperature_range()

def is_temperature_data_fresh(max_age_seconds: float = 5.0) -> bool:
    """便民函数：检查温度数据是否新鲜"""
    storage = get_temperature_storage()
    return storage.is_data_fresh(max_age_seconds)
