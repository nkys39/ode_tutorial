# 20: Walking Pedestrian Simulation

歩行する人のシミュレーション（移動障害物）を学びます。

## 学習内容

- Social Force Modelによる歩行者シミュレーション
- ゴール指向の移動
- 歩行者間の相互作用と回避
- 動的障害物としての歩行者

## 実行

```bash
cd build
make 20_walking_pedestrian
./20_walking_pedestrian/20_walking_pedestrian
```

## Social Force Model

歩行者の動きを物理的な力として モデル化します：

1. **ゴール引力**: 目的地に向かう力
2. **歩行者間斥力**: 他の歩行者を避ける力
3. **障害物斥力**: 壁などを避ける力

```cpp
// ゴールへの引力
F_goal = (v_desired - v_current) / tau

// 他の歩行者からの斥力
F_repulsive = A * exp((2*r - distance) / B)
```

## 可視化

- カプセル: 歩行者
- 黄色い線: 速度ベクトル
- 色: 各歩行者を区別

歩行者はゴールに到達すると、反対方向のゴールに向かって歩き始めます。
