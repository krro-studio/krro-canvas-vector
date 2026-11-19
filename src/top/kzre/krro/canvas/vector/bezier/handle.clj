(ns top.kzre.krro.canvas.vector.bezier.handle
  "Bézier 手柄编辑。

   只做数据处理——不强制约束一致性。约束由 continuity ns 显式应用。

   覆盖的操作：
     1. 设置——直接写入手柄向量 (dx, dy)
     2. 清空——两侧手柄置零
     3. 镜像——单侧手柄按反向复制到对侧（等长反向）
     4. 移动——手柄向量按偏移量平移

   手柄的仿射变换（旋转 / 缩放 / 镜像 / 斜切）由 bezier.transform 完成——
   它作用于整个控制点（位置 + 手柄一起）。本 ns 只管手柄单独编辑。

   仅支持 :bezier 路径。"
  (:require
   [top.kzre.krro.canvas.vector.anchor]
   [top.kzre.krro.canvas.vector.curve :as curve]
   [top.kzre.krro.canvas.vector.path :as path]
   [top.kzre.krro.canvas.vector.anchor :as anchor])
  (:import
   [top.kzre.curve.bezier2d Curve]
   (top.kzre.krro.canvas.vector.anchor Anchor)))

;(defrecord Anchor [path-id point-idx])
(defrecord Handle
  [^Anchor anchor direction]
  "手柄的定位=锚点+方向")

(defn in-handle? [^Handle handle]
  (= (:direction handle) :in))

(defn out-handle? [^Handle handle]
  (= (:direction handle) :out))



;; ═══════════════════════════════════════
;; 内部：校验 / 更新
;; ═══════════════════════════════════════

(defn- check-path [path]
  (when-not (= :bezier (:path-type path))
    (throw (ex-info "handle ops only support :bezier paths"
                    {:path-type (:path-type path)}))))

(defn- check-idx [path idx]
  (let [n (path/path-point-count path)]
    (when-not (<= 0 idx (dec n))
      (throw (ex-info "handle index out of bounds"
                      {:idx idx :point-count n})))))

(defn- update-point-at
  [path idx f]
  (update-in path [:curve :points idx] f))

;; ═══════════════════════════════════════
;; 手柄位置求值
;; ═══════════════════════════════════════

(def ^:private standard-handle-fraction
  "退化手柄沿切向方向的标准长度 = 到邻居控制点距离 × 该比例。
   1/3 是三次 Bézier 逼近圆弧的标准取值。"
  (/ 1.0 3.0))

(defn- neighbor-point
  "取锚点 idx 的前一个（:prev）或后一个（:next）控制点。
   闭合路径在端点处回绕；开路径端点无邻居时返回 nil。"
  [path idx direction]
  (let [points (path/path-points path)
        n      (count points)
        closed (path/path-closed? path)]
    (case direction
      :prev (cond
              (pos? idx)            (nth points (dec idx))
              (and closed (pos? n)) (nth points (dec n))
              :else                 nil)
      :next (cond
              (< idx (dec n))       (nth points (inc idx))
              (and closed (pos? n)) (nth points 0)
              :else                 nil))))

(defn- handle-point*
  "通用手柄位置计算：
   - (dx, dy) 非退化 → (x+dx, y+dy)
   - (dx, dy) 退化   → 沿切向方向（指向 neighbor-dir 的邻居）取标准长度
   - 无邻居（开路径端点）→ 退回锚点本身"
  [path handle dx-key dy-key neighbor-dir]
  (let [idx (:point-idx (:anchor handle))
        pt  (nth (path/path-points path) idx)
        x   (double (:x pt))
        y   (double (:y pt))
        dx  (double (or (get pt dx-key) 0.0))
        dy  (double (or (get pt dy-key) 0.0))]
    (if (> (+ (* dx dx) (* dy dy)) 1e-24)
      {:x (+ x dx) :y (+ y dy)}
      (if-let [nb (neighbor-point path idx neighbor-dir)]
        (let [vx  (- (double (:x nb)) x)
              vy  (- (double (:y nb)) y)
              len (Math/sqrt (+ (* vx vx) (* vy vy)))]
          (if (> len 1e-12)
            (let [s (/ standard-handle-fraction len)]
              {:x (+ x (* vx s))
               :y (+ y (* vy s))})
            {:x x :y y}))
        {:x x :y y}))))


(defn handle-point
  "手柄位置。按 handle 的 :direction 自动分派，不校验方向：

   - :in  → 用 (dx1, dy1)，退化时切向指向上一控制点
   - 其它 → 用 (dx2, dy2)，退化时切向指向下一控制点

   非退化时返回 (x+dx, y+dy)；退化时沿切向取标准长度
   （到邻居距离的 1/3）。

   边界情形：
   - 无锚点            —— 抛异常（check-idx 越界），与 set-handle-* 一致
   - 单锚点 · 开路径   —— 无邻居；非退化正常返回 (x+dx, y+dy)，
                          退化退回锚点本身
   - 单锚点 · 闭路径   —— 邻居回绕为自身，方向向量为零；
                          退化退回锚点本身
   - 退化 · 开路径端点 —— 无邻居，退回锚点本身
   - 退化 · 邻居重合   —— 方向向量长度为 0，退回锚点本身
   - 退化判定          —— dx² + dy² ≤ 1e-24"
  [path handle]
  {:pre [(map? path)]}
  (check-path path)
  (check-idx path (:point-idx (:anchor handle)))
  (if (in-handle? handle)
    (handle-point* path handle :dx1 :dy1 :prev)
    (handle-point* path handle :dx2 :dy2 :next)))

;; ═══════════════════════════════════════
;; 1. 设置
;; ═══════════════════════════════════════

(defn set-handle-in
  "设置锚点 idx 的入切向量 (dx1, dy1)。返回新 path。"
  [path idx dx dy]
  {:pre [(map? path) (int? idx) (number? dx) (number? dy)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   #(assoc % :dx1 (double dx) :dy1 (double dy))))

(defn set-handle-out
  "设置锚点 idx 的出切向量 (dx2, dy2)。返回新 path。"
  [path idx dx dy]
  {:pre [(map? path) (int? idx) (number? dx) (number? dy)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   #(assoc % :dx2 (double dx) :dy2 (double dy))))

;; ═══════════════════════════════════════
;; 2. 清空
;; ═══════════════════════════════════════

(defn clear-handles
  "将锚点 idx 的入切和出切都清零。返回新 path。

   连续性字段不动——由调用方决定是否同步改为 :none。
   若要保持约束一致性，调用方随后需调 continuity 层的 apply-constraints。"
  [path idx]
  {:pre [(map? path) (int? idx)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   #(assoc % :dx1 0.0 :dy1 0.0 :dx2 0.0 :dy2 0.0)))

;; ═══════════════════════════════════════
;; 3. 镜像
;; ═══════════════════════════════════════

(defn mirror-in-from-out
  "入切 ← 出切的反向（等长反向）。返回新 path。

   (dx1, dy1) ← (-dx2, -dy2)。出切保持不变。"
  [path idx]
  {:pre [(map? path) (int? idx)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   (fn [pt]
                     (assoc pt :dx1 (- (double (:dx2 pt)))
                               :dy1 (- (double (:dy2 pt)))))))

(defn mirror-out-from-in
  "出切 ← 入切的反向（等长反向）。返回新 path。

   (dx2, dy2) ← (-dx1, -dy1)。入切保持不变。"
  [path idx]
  {:pre [(map? path) (int? idx)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   (fn [pt]
                     (assoc pt :dx2 (- (double (:dx1 pt)))
                               :dy2 (- (double (:dy1 pt)))))))

;; ═══════════════════════════════════════
;; 4. 移动
;; ═══════════════════════════════════════

(defn move-handle-in
  "平移锚点 idx 的入切向量。返回新 path。

   (dx1, dy1) += (dx, dy)。"
  [path idx dx dy]
  {:pre [(map? path) (int? idx) (number? dx) (number? dy)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   (fn [pt]
                     (assoc pt :dx1 (+ (double (:dx1 pt)) (double dx))
                               :dy1 (+ (double (:dy1 pt)) (double dy))))))

(defn move-handle-out
  "平移锚点 idx 的出切向量。返回新 path。

   (dx2, dy2) += (dx, dy)。"
  [path idx dx dy]
  {:pre [(map? path) (int? idx) (number? dx) (number? dy)]}
  (check-path path)
  (check-idx path idx)
  (update-point-at path idx
                   (fn [pt]
                     (assoc pt :dx2 (+ (double (:dx2 pt)) (double dx))
                               :dy2 (+ (double (:dy2 pt)) (double dy))))))