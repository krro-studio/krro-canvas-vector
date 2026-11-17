(ns top.kzre.krro.canvas.vector.bezier.fit
  "单段拟合。

   调整指定段，使其经过目标点 (x, y)。
   保持端点位置固定，只调整内部控制点 P1 / P2。

   两种拟合：
     fit-segment      —— 指定段内参数 t，两侧手柄沿原方向缩放
     fit-segment-free —— 目标点即段的极点（段内 t=0.5 过点且切线平行于弦），
                         手柄方向也放开
                         （原手柄共线时——挤出的新段——fit 无解，free 才弯得动）

   与 reform 的区别：
     reform —— 从点集重新构造整条（或范围）曲线——控制点数可能变
     fit    —— 单段局部调整形状——控制点数不变

   曲线：Segment.of + Segments.fit / Segments.fitFree（bezier2d Java 工具）。
   采样：width/resample-samples（曲线形状变了——按弧长比例重映射）。
   arc-params 清除。

   仅支持 :bezier 路径。"
  (:require
    [top.kzre.krro.canvas.vector.path   :as path]
    [top.kzre.krro.canvas.vector.curve  :as curve]
    [top.kzre.krro.canvas.vector.width  :as width])
  (:import
    (top.kzre.curve.bezier2d Segment Segments)))

;; ═══════════════════════════════════════
;; 内部：新段写回
;; ═══════════════════════════════════════

(defn- require-bezier-path
  [op path]
  (when-not (= :bezier (:path-type path))
    (throw (ex-info (str op " only supports :bezier paths")
                    {:path-type (:path-type path)}))))

(defn- fit-with
  "取段 seg-idx 的 Segment，交给 adjust 算出新段，写回 path。

   adjust 是 Segment -> Segment——两个公开 API 只差在这一步。
   点数不变，故采样按弧长比重映射。"
  [path seg-idx adjust]
  (let [curve-edn (:curve path)
        points    (:points curve-edn)
        n         (count points)
        seg-count (dec n)]
    (when-not (<= 0 seg-idx (dec seg-count))
      (throw (ex-info "seg-idx out of range"
                      {:seg-idx seg-idx :seg-count seg-count})))
    (let [jc      (curve/edn->curve curve-edn)
          jpoints (.getPoints jc)
          seg     (Segment/of (nth jpoints seg-idx) (nth jpoints (inc seg-idx)))
          new-seg (adjust seg)

          sa (.getA new-seg)
          sb (.getB new-seg)
          sc (.getC new-seg)
          sd (.getD new-seg)

          new-p1 (-> (nth points seg-idx)
                     (assoc :x   (.getX sa)
                            :y   (.getY sa)
                            :dx2 (- (.getX sb) (.getX sa))
                            :dy2 (- (.getY sb) (.getY sa))))
          new-p2 (-> (nth points (inc seg-idx))
                     (assoc :x   (.getX sd)
                            :y   (.getY sd)
                            :dx1 (- (.getX sc) (.getX sd))
                            :dy1 (- (.getY sc) (.getY sd))))

          new-points    (-> points
                            (assoc seg-idx new-p1)
                            (assoc (inc seg-idx) new-p2))
          new-curve-edn (assoc curve-edn :points new-points)

          width-type    (path/path-width-type path)
          changes       (width/resample-samples width-type
                                                (:width-samples path) (:t-params path)
                                                curve-edn new-curve-edn
                                                n)]   ; 点数不变

      (-> path
          (assoc :curve new-curve-edn)
          (dissoc :arc-params)
          (width/apply-samples width-type changes)))))

;; ═══════════════════════════════════════
;; 公开 API
;; ═══════════════════════════════════════

(defn fit-segment
  "调整段 seg-idx，使其在段内参数 local-t 处经过目标点 (x, y)。
   两侧手柄只沿原方向缩放——原手柄共线（直线段）时无解。

   返回新 path。控制点数不变。
   采样按弧长比重映射；arc-params 清除。"
  [path seg-idx local-t x y]
  {:pre [(map? path) (int? seg-idx) (number? local-t)
         (number? x) (number? y)]}
  (require-bezier-path "fit-segment" path)
  (when-not (<= 0.0 (double local-t) 1.0)
    (throw (ex-info "local-t out of [0, 1]"
                    {:local-t local-t})))
  (fit-with path seg-idx
            #(Segments/fit % (double local-t) (double x) (double y))))

(defn fit-segment-free
  "调整段 seg-idx，使其自然地经过目标点 (x, y)——目标点就是段的极点。

   极点 = 离弦（两端点连线）最远之点：段内 t=0.5 处精确经过目标点，
   且该点切线平行于弦。两端点不动，两个手柄的方向和长度全部重排——
   不受原手柄约束，也不管锚点的连续性。
   弧不越过弦；目标点在弦线上时退化为直线段。

   返回新 path。控制点数不变。
   采样按弧长比重映射；arc-params 清除。"
  [path seg-idx x y]
  {:pre [(map? path) (int? seg-idx) (number? x) (number? y)]}
  (require-bezier-path "fit-segment-free" path)
  (fit-with path seg-idx #(Segments/fitFree % (double x) (double y))))
