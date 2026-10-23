using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// ORCA（最优互惠碰撞避免）求解器：与系统解耦的纯算法模块。
    /// 输入是自己的位置/速度/半径、邻居列表与期望速度，输出是避障后的新速度。
    /// Burst 兼容：不持有任何 static 字段，宿主的 ComponentLookup 由调用方传入。
    /// </summary>
    public static class OrcaSolver
    {
        /// <summary>
        /// 检测线段是否与圆相交（用于 ORCA 调试，当前未在主流程使用）
        /// </summary>
        public static bool LineInsideCircle(float _cirlceRadius, float2 _circleCenter, OrcaLine _line, float _lineLength)
        {
            if (math.distancesq(_circleCenter, _line.Point + _line.Direction * _lineLength) < math.pow(_cirlceRadius, 2))
                return true;

            float radiusSq = math.pow(_cirlceRadius, 2);

            if (math.distancesq(_circleCenter, _line.Point) < radiusSq)
                return true;

            float2 centerToLineStart = _line.Point - _circleCenter;
            float2 directionWLength = _line.Direction * _lineLength;

            float a = math.dot(directionWLength, directionWLength);
            float b = 2 * math.dot(centerToLineStart, directionWLength);
            float c = math.dot(centerToLineStart, centerToLineStart) - radiusSq;

            if (a < float.Epsilon) return false;

            float d = math.pow(b, 2) - 4 * a * c;
            if (d < 0) return false;

            float sqrtD = math.sqrt(d);
            float x1 = (-b - sqrtD) / (2 * a);
            float x2 = (-b + sqrtD) / (2 * a);

            return (x1 <= 1 && x2 >= 0);
        }

        /// <summary>
        /// 将二维向量按指定弧度角旋转（用于 ORCA 切线方向计算）
        /// </summary>
        public static float2 RotateFloat2(float2 _vector, float _radAngle)
        {
            return new float2(_vector.x * math.cos(_radAngle) - _vector.y * math.sin(_radAngle),
                _vector.x * math.sin(_radAngle) + _vector.y * math.cos(_radAngle));
        }

        /// <summary>
        /// ORCA（最优互惠碰撞避免）核心算法：为当前代理计算考虑所有邻居后的最优速度。
        ///
        /// 算法原理：
        /// 对每个邻居代理：
        ///   - 计算相对位置向量 relativePosition（其他代理位置 - 当前代理位置）
        ///   - 计算相对速度向量 relativeVelocity（当前代理速度 - 其他代理速度）
        ///   - 情况1：代理未重叠（distSq > combinedRadiusSq）
        ///     - 在时间窗口 timeHorizon 内计算速度障碍（Velocity Obstacle）的截锥
        ///     - 若当前相对速度在锥外：正常计算 ORCA 切线方向和偏移量 u
        ///     - 若当前相对速度在锥内（圆心方向）：沿圆弧法线方向逃逸
        ///   - 情况2：代理已重叠（distSq <= combinedRadiusSq）
        ///     - 使用当前帧时间步长代替 timeHorizon（紧急避让）
        ///     - 立即计算最小分离速度
        ///   - ORCA 线：point = 当前速度 + u/2（互惠各承担一半责任）
        ///              direction = 垂直于 u 的方向（约束线方向）
        ///
        /// 速度优化：
        ///   调用 FindSubOptimalPath 在所有 ORCA 约束线形成的半平面集合中
        ///   用线性规划找最接近理想速度（方向×最大速度）的合法速度
        /// </summary>
        public static void CalculateFinalVelocity(
            NativeList<OrcaLine> _orcaLines,
            ComponentLookup<Agent> _agents,
            ComponentLookup<LocalTransform> _transforms, 
            Agent _agent, 
            AvoidanceAgent _avoidanceAgent,
            LocalTransform _agentLocalTrasnfrom, 
            float3 _currentPathPosition,
            DynamicBuffer<NeighbourBuffer> _agentNeighbours,
            float deltaTime, 
            float2 _optimalVel, 
            ref float2 _newVelocity)
        {
            int numObstLines = _orcaLines.Length;
            float invTimeHorizon = 1.0f / _avoidanceAgent.TimeHorizon; // 1/timeHorizon

            // 遍历所有邻居，生成对应的 ORCA 约束线
            for (int i = 0; i < _agentNeighbours.Length; ++i)
            {
                Entity otherAgentEntity = _agentNeighbours[i].Agent;

                // 2D 相对位置（XZ 平面）：其他代理 → 当前代理
                float2 relativePosition = V2(_transforms[otherAgentEntity].Position)
                    - V2(_agentLocalTrasnfrom.Position);

                // 相对速度：当前代理速度 - 其他代理速度
                float2 relativeVelocity = V2(_agent.CurrentVelocity)
                    - V2(_agents[otherAgentEntity].CurrentVelocity);

                float distSq = math.pow(math.length(relativePosition), 2);
                float combinedRadius = _agent.Radius + _agents[otherAgentEntity].Radius;
                float combinedRadiusSq = math.pow(combinedRadius, 2);

                OrcaLine line;
                float2 u; // 速度修正量（从当前相对速度到 ORCA 边界的最短向量）

                if (distSq > combinedRadiusSq)
                {
                    // ===== 代理未重叠：标准 ORCA 计算 =====
                    // w = 相对速度 - (1/timeHorizon) * 相对位置（在截锥中的位移向量）
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    float wLengthSq = math.pow(math.length(w), 2);
                    float dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && math.pow(dotProduct1, 2) > combinedRadiusSq * wLengthSq)
                    {
                        // w 指向截锥圆底：沿圆弧法线方向（最短逃逸到圆外）
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.Direction = new float2(unitW.y, -unitW.x); // 切线方向（垂直 unitW）
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW; // 推出圆的修正量
                    }
                    else
                    {
                        // w 在截锥腿外：沿截锥腿方向（最短逃逸到截锥外）
                        float leg = math.sqrt(distSq - combinedRadiusSq); // 切线长度

                        if (math.determinant(new float2x2(relativePosition, w)) > 0.0f)
                        {
                            // w 在左腿外侧：计算左截锥腿的方向
                            line.Direction = new float2(
                                relativePosition.x * leg - relativePosition.y * combinedRadius,
                                relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            // w 在右腿外侧：计算右截锥腿的方向
                            line.Direction = -new float2(
                                relativePosition.x * leg + relativePosition.y * combinedRadius,
                                -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                        u = dotProduct2 * line.Direction - relativeVelocity; // 沿约束线方向的投影修正
                    }
                }
                else
                {
                    // ===== 代理已重叠：紧急避让 =====
                    // 使用当前时间步长（而非 timeHorizon）立即分离
                    float invTimeStep = 1.0f / deltaTime;

                    float2 w = relativeVelocity - invTimeStep * relativePosition;
                    float wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                // ORCA 线的参考点：当前速度 + 修正量的一半（互惠：双方各承担一半避让责任）
                line.Point = V2(_agent.CurrentVelocity) + 0.5f * u;
                _orcaLines.Add(line);
            }

            // ===== 线性规划求最优速度 =====
            // FindSubOptimalPath：在速度圆（半径=MaxSpeed）和所有 ORCA 半平面约束下
            // 找最接近理想速度（optimalVel * MaxSpeed）的合法速度
            int lineFail = FindSubOptimalPath(_orcaLines, _agent.MaxSpeed, _optimalVel * _agent.MaxSpeed, false, ref _newVelocity);

            // 若第一轮线性规划有约束线无法满足，使用 3D 线性规划（FindSubOptimalPathWithLines）处理
            if (lineFail < _orcaLines.Length)
            {
                FindSubOptimalPathWithLines(_orcaLines, numObstLines, lineFail, _agent.MaxSpeed, ref _newVelocity);
            }
        }

        /// <summary>
        /// 在一条 ORCA 约束线上寻找满足所有之前约束的最优点（2D 线性规划子问题）。
        ///
        /// 算法步骤：
        /// 1. 将约束线与速度圆（radius）求交，得到参数范围 [tLeft, tRight]
        /// 2. 遍历之前所有约束线，进一步缩小参数范围（半平面交集）
        /// 3. 在合法范围内找最接近 optVelocity 的点（投影到线上）
        /// </summary>
        public static bool FindOptimalPath(NativeList<OrcaLine> lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            float dotProduct = math.dot(lines[lineNo].Point, lines[lineNo].Direction);
            // 判别式：约束线与速度圆的交点参数
            float discriminant = math.pow(dotProduct, 2) + math.pow(radius, 2) - math.pow(math.length(lines[lineNo].Point), 2);

            if (discriminant < 0.0f)
            {
                return false; // 约束线完全在速度圆外，无解
            }

            float sqrtDiscriminant = math.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;   // 与速度圆的左交点参数
            float tRight = -dotProduct + sqrtDiscriminant;  // 与速度圆的右交点参数

            // 用之前的每条约束线进一步缩小合法参数范围
            for (int i = 0; i < lineNo; ++i)
            {
                // 计算两条线的叉积（行列式）：判断是否平行
                float denominator = math.determinant(new float2x2(lines[lineNo].Direction, lines[i].Direction));
                // 计算两条线的交点参数
                float numerator = math.determinant(new float2x2(lines[i].Direction, lines[lineNo].Point - lines[i].Point));

                if (math.abs(denominator) <= 0.00001f)
                {
                    // 两线平行：检查是否同向（平行且互相满足则跳过，否则无解）
                    if (numerator < 0.0f)
                    {
                        return false;
                    }
                    continue;
                }

                float t = numerator / denominator; // 交点在当前约束线上的参数

                if (denominator >= 0.0f)
                {
                    tRight = math.min(tRight, t); // 缩小右边界
                }
                else
                {
                    tLeft = math.max(tLeft, t);  // 缩小左边界
                }

                if (tLeft > tRight)
                {
                    return false; // 合法范围为空，无解
                }
            }

            // 在合法范围 [tLeft, tRight] 内选择最优点
            if (directionOpt)
            {
                // 方向优化模式：选择最接近目标方向的端点
                if (math.dot(optVelocity, lines[lineNo].Direction) > 0.0f)
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                else
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
            }
            else
            {
                // 速度优化模式：将目标速度投影到线上，然后 clamp 到合法范围
                float t = math.dot(lines[lineNo].Direction, optVelocity - lines[lineNo].Point);

                if (t < tLeft)
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
                else if (t > tRight)
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                else
                    result = lines[lineNo].Point + t * lines[lineNo].Direction;
            }

            return true;
        }

        /// <summary>
        /// 第一阶段线性规划：在速度圆和 ORCA 约束半平面集合中找最优速度（O(n²) 增量算法）。
        ///
        /// 思路：
        /// - 初始结果 = 理想速度（若超出速度圆则截断）
        /// - 逐条添加约束线：若当前结果违反该约束，在该约束线上重新求最优点
        /// - 若无法满足某条约束（FindOptimalPath 返回 false），返回失败的约束线索引
        ///
        /// 返回值：若所有约束均满足则返回 lines.Length；否则返回失败的约束线索引
        /// </summary>
        public static int FindSubOptimalPath(NativeList<OrcaLine> lines, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            if (directionOpt)
            {
                result = optVelocity * radius; // 方向优化：速度大小 = radius
            }
            else if (math.pow(math.length(optVelocity), 2) > math.pow(radius, 2))
            {
                result = math.normalize(optVelocity) * radius; // 超出速度圆：截断到圆上
            }
            else
            {
                result = optVelocity; // 理想速度在速度圆内：直接使用
            }

            for (int i = 0; i < lines.Length; ++i)
            {
                // 检查当前结果是否违反约束线 i（叉积 > 0 表示在错误侧）
                if (math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result)) > 0.0f)
                {
                    float2 tempResult = result;

                    if (!FindOptimalPath(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult; // 恢复之前的结果
                        return i;           // 返回失败的约束线索引
                    }
                }
            }

            return lines.Length; // 所有约束均满足
        }

        /// <summary>
        /// 第二阶段线性规划（3D 线性规划）：处理第一阶段无法满足的约束（过度约束情况）。
        ///
        /// 当第一阶段失败时，意味着存在约束冲突（所有 ORCA 约束同时满足是不可能的）。
        /// 此阶段通过将约束投影到更高维度，找到违反约束最少的"最优次优"速度。
        ///
        /// 算法：
        /// 对每条未能满足的约束线，将之前所有约束线投影到该约束线上，
        /// 在投影约束下用方向优化模式重新求解，最小化违反约束的程度
        /// </summary>
        public static void FindSubOptimalPathWithLines(NativeList<OrcaLine> lines, int numObstLines, int beginLine, float radius, ref float2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Length; ++i)
            {
                // 检查当前结果违反约束的程度（行列式值 > distance 表示更严重的违反）
                if (math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result)) > distance)
                {
                    // 将之前所有约束线投影到当前约束线 i 上
                    NativeList<OrcaLine> projLines = new NativeList<OrcaLine>(Allocator.Temp);

                    // 先复制障碍物约束线（不投影，保持不变）
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    // 将代理约束线逐条投影到当前约束线 i
                    for (int j = numObstLines; j < i; ++j)
                    {
                        OrcaLine line;

                        float determinant = math.determinant(new float2x2(lines[i].Direction, lines[j].Direction));

                        if (math.abs(determinant) <= 0.00001f)
                        {
                            // 两线平行：同向则忽略，反向则取中点
                            if (math.dot(lines[i].Direction, lines[j].Direction) > 0.0f)
                            {
                                continue; // 同向平行：约束线 j 被 i 覆盖
                            }
                            else
                            {
                                line.Point = 0.5f * (lines[i].Point + lines[j].Point); // 反向平行：取中间点
                            }
                        }
                        else
                        {
                            // 非平行：计算两线交点（约束线 j 在约束线 i 上的投影点）
                            line.Point = lines[i].Point + (math.determinant(new float2x2(lines[j].Direction, lines[i].Point - lines[j].Point)) / determinant) * lines[i].Direction;
                        }

                        // 投影线方向：两原始约束线方向的差的单位化
                        line.Direction = math.normalize(lines[j].Direction - lines[i].Direction);
                        projLines.Add(line);
                    }

                    float2 tempResult = result;

                    // 在投影约束下用方向优化重新求解（沿约束线 i 的法线方向）
                    if (FindSubOptimalPath(projLines, radius, new float2(-lines[i].Direction.y, lines[i].Direction.x), true, ref result) < projLines.Length)
                    {
                        result = tempResult; // 若投影求解也失败，则保持原结果
                    }

                    // 更新当前违反约束的程度
                    distance = math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result));

                    projLines.Dispose();
                }
            }
        }

        /// <summary>取 XZ 分量：ORCA 只在水平平面（XZ）计算（原 PathfindingUtility.V2FromV3）。</summary>
        public static float2 V2(float3 _v)
        {
            return new float2(_v.x, _v.z);
        }
    }
}
