/**
 * DOM 与格式化小工具：全前端只有这一层可以假设"元素一定存在"。
 *
 * 约定：只在模块初始化时取一次元素的地方（如摇杆）直接 getElementById；
 * 每次渲染都取的地方也用 $，因为这里的页面结构是静态的，查表代价可以忽略。
 */

/** 按 id 取元素。 */
export const $ = (id) => document.getElementById(id);

/**
 * 数值格式化为固定小数位；null/undefined/NaN 统一显示 "—"。
 * 后端字段缺失时不能显示 0.000，那会被误读成"实测为零"。
 */
export function fixed(value, digits) {
  if (value === null || value === undefined || Number.isNaN(value)) return "—";
  return Number(value).toFixed(digits);
}
