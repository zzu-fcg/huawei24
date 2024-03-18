/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 */

package com.huawei.codecraft;

import javax.lang.model.element.VariableElement;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

/**
 * 思路：
 * <p>1.规划机器人路线获取货物
 * <p>2.规划船舶路线到达港口(初赛：船舶到哪个港口)
 */
public class Main {
    private static final int n = 200;
    private static final int robot_num = 10;
    private static final int berth_num = 10;
    private static final int boat_num = 5;
    private static final int N = 210;
    private static final String fake_path = "[-1, -1]";

    private int money, boat_capacity, id;
    private String[] ch = new String[N];
    /**
     * ‘.’ ： 空地 <p>
     * ‘*’ ： 海洋 <p>
     * ‘#’ ： 障碍 <p>
     * ‘A’ ： 机器人起始位置，总共 10 个 <p>
     * ‘B’ ： 大小为 4*4，表示泊位的位置,泊位标号在后泊位处初始化
     */
    private char[][] map = new char[N][N];
    private boolean[][] gds_map = new boolean[N][N];
    private List<Good> gds = new LinkedList<>(); //ArrayList or LinkedList
    private Robot[] robots = new Robot[robot_num + 10];
    private Berth[] berth = new Berth[berth_num + 10];
    private Boat[] boat = new Boat[10];
    private StringBuilder msg = new StringBuilder();

    /**
     * 信号量，限制子线程数量最多一个
     */
    //private static boolean semaphore = true;
    private static boolean[] semaphore = new boolean[robot_num];

    /**
     * 固定大小线程池
     */
    private ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(2);

    private void init() {
        Scanner scanf = new Scanner(System.in);
        for (int i = 1; i <= n; i++) {
            ch[i] = scanf.nextLine();
            map[i - 1] = ch[i].toCharArray();
            Arrays.fill(gds_map[i - 1], false);
        }
        for (int i = 0; i < berth_num; i++) {
            int id = scanf.nextInt();
//            berth[id] = new Berth();
//            berth[id].x = scanf.nextInt();
//            berth[id].y = scanf.nextInt();
//            berth[id].transport_time = scanf.nextInt();
//            berth[id].loading_speed = scanf.nextInt();
            int x = scanf.nextInt();
            int y = scanf.nextInt();
            int transport_time = scanf.nextInt();
            int loading_speed = scanf.nextInt();
            berth[id] = new Berth(x, y, transport_time, loading_speed);
        }
        this.boat_capacity = scanf.nextInt();
        String okk = scanf.nextLine();
        System.out.println("OK");
        System.out.flush();
        //初始化线程池
        executor.execute(() -> {

        });
    }

    private int input() {
        Scanner scanf = new Scanner(System.in);
        this.id = scanf.nextInt();
        this.money = scanf.nextInt();
        int num = scanf.nextInt();
        for (int i = 1; i <= num; i++) {
            int x = scanf.nextInt();
            int y = scanf.nextInt();
            int val = scanf.nextInt();
            gds.add(new Good(x, y, val));
            gds_map[x][y] = true;
        }
        for (int i = 0; i < robot_num; i++) {
            if (robots[i] == null) {
                robots[i] = new Robot();
            }
            robots[i].goods = scanf.nextInt();
            robots[i].x = scanf.nextInt();
            robots[i].y = scanf.nextInt();
            int sts = scanf.nextInt();
        }
        for (int i = 0; i < 5; i++) {
            if (boat[i] == null) {
                boat[i] = new Boat();
            }
            boat[i].status = scanf.nextInt();
            boat[i].pos = scanf.nextInt();
        }
        String okk = scanf.nextLine();
        return id;
    }

    /**
     * 机器人取货
     *
     * @param robot 机器人
     */
    private void robot2Good(Robot robot, StringBuilder msg) {
        int min_dist = 200;
        int rx = robot.x, ry = robot.y;
        //todo 遍历货物寻找最优路径(优化：最有价值)
        Good best = null;
        for (Good gd : gds) {
            if (!gds_map[gd.x][gd.y]) {
                continue;
            }
            if (min_dist < 50) {
                Astar astar = new Astar(map, gds_map);
                robot.path = astar.aStarSearchGood(new int[]{rx, ry}, new int[]{best.x, best.y}, msg);
                //路径不空且时间足够到达，更新货物生存时间,并记录返回路径
                if (!robot.path.isEmpty() && !Arrays.toString(robot.path.peek()).equals(fake_path) && best.left_time > robot.path.size()) {
                    int[] end = robot.path.peekLast();
                    gds_map[end[0]][end[1]] = false;
                    markPath(robot);
                }
                return;
            }
            int dist = Math.abs(gd.x - rx) + Math.abs(gd.y - ry);
            if (dist < min_dist) {
                min_dist = dist;
                best = gd;
            }
        }
        if (best != null && robot.path.isEmpty()) {
            Astar astar = new Astar(map, gds_map);
            robot.path = astar.aStarSearchGood(new int[]{rx, ry}, new int[]{best.x, best.y}, msg);
        }
        if (!robot.path.isEmpty() && !Arrays.toString(robot.path.peek()).equals(fake_path) && best.left_time > robot.path.size()) {
            int[] end = robot.path.peekLast();
            gds_map[end[0]][end[1]] = false;
            markPath(robot);
        }
    }

    /**
     * 拿到货物的机器人到港口
     *
     * @param robot 机器人
     */
    private void robot2Berth(Robot robot) {
        if (inBerth(robot)) {
            return;
        }
        Random rand = new Random();
        int rx = robot.x, ry = robot.y;
        //遍历港口寻找最优路径
        Queue<int[]> dist_que = new PriorityQueue<int[]>(Comparator.comparingInt(e -> e[1]));
        for (int j = 0; j < berth_num; j++) {
            int dist = Math.abs(berth[j].x - rx) + Math.abs(berth[j].y - ry);
            dist_que.add(new int[]{j, dist});
        }
        while (!dist_que.isEmpty()) {
            int[] pair = dist_que.poll();
            int num = pair[0];
            Astar astar = new Astar(map);
            robot.path = astar.aStarSearchBerth(new int[]{rx, ry}, new int[]{berth[num].x + rand.nextInt(4), berth[num].y + rand.nextInt(4)});
            if (robot.path != null && !fake_path.equals(Arrays.toString(robot.path.peek()))) {
                break;
            }
        }
        markPath(robot);
    }

    private boolean inBerth(Robot robot) {
        int rx = robot.x;
        int ry = robot.y;
        for (int i = 0; i < berth_num; i++) {
            int x = berth[i].x;
            int y = berth[i].y;
            if (rx >= x && rx <= x + 3 && ry >= y && ry <= y + 3) {
                return true;
            }
        }
        return false;
    }

    /**
     * 对机器人路径进行标记，使得a*规划路径时可以绕过已规划的路径，降低机器人碰撞概率
     *
     * @param robot
     */
    private void markPath(Robot robot) {
        if (robot.path == null || fake_path.equals(Arrays.toString(robot.path.peek()))) {
            return;
        }
        if (!robot.path_mark.isEmpty()) {
            map[robot.x][robot.y] = robot.path_mark.poll();
            robot.path_mark.clear();
        }
        for (int[] point : robot.path) {
            robot.path_mark.add(map[point[0]][point[1]]);
            map[point[0]][point[1]] = '^';
        }
    }

    /**
     * bfs
     *
     * @param begin 起点坐标
     * @param end   终点坐标
     * @return 路径队列
     */
    public Queue<int[]> getPath(int[] begin, int[] end) {
        //移动的四个方向
        int[][] dirs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        //储存未进行处理的点
        Queue<int[]> que = new LinkedList<int[]>();
        Queue<int[]> path = new LinkedList<int[]>();
        Map<String, int[]> father = new HashMap<String, int[]>();
        //记录是否访问过该位置
        boolean[][] visited = new boolean[n][n];
        //将起始点放入队列
        que.offer(begin);
        //一直循环直到队列为空
        while (!que.isEmpty()) {
            //取出队列中最前端的点
            int[] current = que.poll();
            //标记为已访问
            visited[current[0]][current[1]] = true;
            //如果是终点,记录路径
            if (current[0] == end[0] && current[1] == end[1]) {
                while (current != null) {
                    path.add(current);
                    current = father.get(Arrays.toString(current));
                }
                Collections.reverse((List) path);
                return path;
            }
            //四个方向循环
            for (int[] dir : dirs) {
                int nx = current[0] + dir[0];
                int ny = current[1] + dir[1];
                //判断是否可以走
                if (ny >= 0 && ny < map[0].length && nx >= 0 && nx < map.length && map[nx][ny] == '.' && !visited[nx][ny]) {
                    //并将该点放入队列等待下次处理
                    que.offer(new int[]{nx, ny});
                    //记录该点的父节点
                    father.put(Arrays.toString(new int[]{nx, ny}), new int[]{current[0], current[1]});
                }
            }
        }
        return null;
    }

    private void getRobotsCmd(Robot robot, int idx, StringBuilder msg) {
        int[] up = {-1, 0};
        int[] down = {1, 0};
        int[] left = {0, -1};
        int[] right = {0, 1};
        Queue<int[]> path = robot.path;
        int[] now;
        if (path.size() > 1) {
            now = path.poll();
//            if (gds_map[now[0]][now[1]] && robot.goods == 0) {
//                //路途中提前找到货物
//                System.out.printf("get %d" + System.lineSeparator(), idx);
//                //清除剩余标记
//                while (robot.path.size() > 1 && robot.path_mark.size() > 1) {
//                    int[] point = robot.path.pollLast();
//                    if (!robot.path_mark.isEmpty()) {
//                        map[point[0]][point[1]] = robot.path_mark.pollLast();
//                    }
//                }
//                robot.path.poll();
//                return;
//            }
            if (!robot.path_mark.isEmpty()) {
                map[now[0]][now[1]] = robot.path_mark.poll();
            }
            int[] next = path.peek();
            if (next != null) {
                int[] dir = {next[0] - now[0], next[1] - now[1]};
                if (Arrays.equals(dir, right)) {
                    System.out.printf("move %d %d" + System.lineSeparator(), idx, 0);
                } else if (Arrays.equals(dir, left)) {
                    System.out.printf("move %d %d" + System.lineSeparator(), idx, 1);
                } else if (Arrays.equals(dir, up)) {
                    System.out.printf("move %d %d" + System.lineSeparator(), idx, 2);
                } else if (Arrays.equals(dir, down)) {
                    System.out.printf("move %d %d" + System.lineSeparator(), idx, 3);
                }
            }
        }
        if (path.size() == 1) {
            now = path.poll();
//            if (!robot.path_mark.isEmpty()) {
//                map[now[0]][now[1]] = robot.path_mark.poll();
//            }
            if (robot.goods == 0) { //无货取货
                System.out.printf("get %d" + System.lineSeparator(), idx);
            } else { //有货卸货
                System.out.printf("pull %d" + System.lineSeparator(), idx);
                for (int i = 0; i < berth_num; i++) {
                    int x = berth[i].x;
                    int y = berth[i].y;
                    if (now[0] >= x && now[0] <= x + 3 && now[1] >= y && now[1] <= y + 3) {
                        berth[i].goods_num++;
                        break;
                    }
                }
            }
        }
//        if (!robot.path.isEmpty()) {
//            Queue<int[]> path = robot.path;
//            int[] now;
//            if (path.size() > 1) {
//                now = path.poll();
//                if (!robot.path_mark.isEmpty() && robot.back_path.isEmpty()) {
//                    char c = robot.path_mark.poll();
//                    if (!robot.arrived && c == 'B') {
//                        robot.arrived = true;
//                    }
//                    map[now[0]][now[1]] = c;
//                }
//                int[] next = path.peek();
//                if (next != null) {
//                    int[] dir = {next[0] - now[0], next[1] - now[1]};
//                    if (Arrays.equals(dir, right)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 0);
//                    } else if (Arrays.equals(dir, left)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 1);
//                    } else if (Arrays.equals(dir, up)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 2);
//                    } else if (Arrays.equals(dir, down)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 3);
//                    }
//                }
//            }
//            if (path.size() == 1) {
//                now = path.poll();
//                if (!robot.path_mark.isEmpty() && robot.back_path.isEmpty()) {
//                    char c = robot.path_mark.poll();
//                    if (!robot.arrived && c == 'B') {
//                        robot.arrived = true;
//                    }
//                    map[now[0]][now[1]] = c;
//                }
//                if (robot.goods == 0) { //无货取货
//                    System.out.printf("get %d" + System.lineSeparator(), idx);
//                } else { //有货卸货
//                    System.out.printf("pull %d" + System.lineSeparator(), idx);
//                }
//            }
//        } else if (!robot.back_path.isEmpty()) {
//            Queue<int[]> path = robot.back_path;
//            int[] now;
//            if (path.size() > 1) {
//                now = path.poll();
//                if (!robot.path_mark.isEmpty()) {
//                    map[now[0]][now[1]] = robot.path_mark.pollLast();
//                }
//                int[] next = path.peek();
//                if (next != null) {
//                    int[] dir = {next[0] - now[0], next[1] - now[1]};
//                    if (Arrays.equals(dir, right)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 0);
//                    } else if (Arrays.equals(dir, left)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 1);
//                    } else if (Arrays.equals(dir, up)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 2);
//                    } else if (Arrays.equals(dir, down)) {
//                        System.out.printf("move %d %d" + System.lineSeparator(), idx, 3);
//                    }
//                }
//            }
//            if (path.size() == 1) {
//                now = path.poll();
//                if (!robot.path_mark.isEmpty()) {
//                    map[now[0]][now[1]] = robot.path_mark.pollLast();
//                }
//                //卸货
//                System.out.printf("pull %d" + System.lineSeparator(), idx);
//            }
//        }
    }

    private int[] shipBoat() {
        int[] res = new int[5];
        Map<Integer, Integer> berth_map = new HashMap<>();
        for (int i = 0; i < 10; i++) {
            berth_map.put(i, berth[i].goods_num);
        }
        List<Map.Entry<Integer, Integer>> list = new ArrayList<Map.Entry<Integer, Integer>>(berth_map.entrySet());
        list.sort((o1, o2) -> o2.getValue() - o1.getValue());
        for (int i = 0; i < 5; i++) {
            int idx = list.get(i).getKey();
            System.out.printf("ship %d %d" + System.lineSeparator(), i, idx);
            res[i] = idx;
        }
        return res;
    }

    private void goBoat(int zhen, int[] berth_idx) {
        for (int i = 0; i < 5; i++) {
            System.out.printf("go %d" + System.lineSeparator(), i);
        }
        int time = 0;
        if (zhen == 4200) {
            time = 3700;
        } else if (zhen == 10200) {
            time = 4300;

        } else {
            time = 1700;
        }
        for (int i = 0; i < 5; i++) {
            int load_num = time / berth[berth_idx[i]].loading_speed;
            int num = berth[berth_idx[i]].goods_num;
            if (load_num >= num)
                berth[berth_idx[i]].goods_num = 0;
            else
                berth[berth_idx[i]].goods_num = num - load_num;
        }
    }

    /**
     * 机器人计划
     *
     * @param robot 机器人
     */
    private void robotPlan(Robot robot) {
//        if (!robot.arrived) {
        if (robot.goods == 0) {
            //取货路径规划
            robot2Good(robot, msg);
        } else {
            //送货路径规划
            //if (!fake_path.equals(Arrays.toString(robot.path.peek())))
            robot2Berth(robot);
        }
//        } else {
//            if (robot.goods == 0) {
//                //取货路径规划,并且原路返回港口
//                robot2Good(robot);
//            }
//        }
    }

    private void collision_avoidance() {
        /**
         * 1.取出前俩点
         * 2.判断是否碰撞
         * 3.检测后路径插入
         */
        int[] up = {-1, 0};
        int[] down = {1, 0};
        int[] left = {0, -1};
        int[] right = {0, 1};
        int[] stay = {0, 0};
        List<List<int[]>> points = new ArrayList<>();
        List<Integer> idxs = new ArrayList<>();
        for (int i = 0; i < robot_num; i++) {
            if (robots[i].path.isEmpty() || fake_path.equals(Arrays.toString(robots[i].path.peek()))) {
                continue;
            }
            List<int[]> p = new ArrayList<>();
            p.add(robots[i].path.pollFirst());
            p.add(robots[i].path.pollFirst());
            points.add(p);
            idxs.add(i);
        }

        int length = points.size();
        for (int i = 1; i < length; i++) {
            List<int[]> robot_i_points = points.get(i);
            int[] robot_i_point = robot_i_points.get(1);
            for (int j = 0; j < i; j++) {
                List<int[]> robot_j_points = points.get(i);
                int[] avoid_point = robot_j_points.get(1);
                if (avoid_point[0] == robot_i_point[0] && avoid_point[1] == robot_i_point[1]) {
                    int x = robot_i_points.get(0)[0];
                    int y = robot_i_points.get(0)[1];
                    int[] dir = new int[]{robot_i_point[0] - x, robot_i_point[1] - y};
                    try {
                        if (Arrays.equals(dir, up)) {
                            if (0 <= y - 1 && map[x][y - 1] != '#' && map[x][y - 1] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y - 1});
                                robot_i_points.add(1, new int[]{x, y - 1});
                            } else if (y + 1 < 200 && map[x][y + 1] != '#' && map[x][y + 1] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y + 1});
                                robot_i_points.add(1, new int[]{x, y + 1});
                            } else {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                            }
                        } else if (Arrays.equals(dir, down)) {
                            if (y + 1 < 200 && map[x][y + 1] != '#' && map[x][y + 1] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y + 1});
                                robot_i_points.add(1, new int[]{x, y + 1});
                            } else if (0 <= y - 1 && map[x][y - 1] != '#' && map[x][y - 1] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y - 1});
                                robot_i_points.add(1, new int[]{x, y - 1});
                            } else {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                            }
                        } else if (Arrays.equals(dir, left)) {
                            if (x + 1 < 200 && map[x + 1][y] != '#' && map[x + 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                            } else if (0 <= x - 1 && map[x - 1][y] != '#' && map[x - 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                            } else {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y + 1});
                                robot_i_points.add(1, new int[]{x, y + 1});
                            }
                        } else if (Arrays.equals(dir, right)) {
                            if (0 <= x - 1 && map[x - 1][y] != '#' && map[x - 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                            } else if (x + 1 < 200 && map[x + 1][y] != '#' && map[x + 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                            } else {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y - 1});
                                robot_i_points.add(1, new int[]{x, y - 1});
                            }
                        } else if (Arrays.equals(dir, stay)) {
                            if (0 <= x - 1 && map[x - 1][y] != '#' && map[x - 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                                robot_i_points.add(1, new int[]{x - 1, y});
                            } else if (x + 1 < 200 && map[x + 1][y] != '#' && map[x + 1][y] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                                robot_i_points.add(1, new int[]{x + 1, y});
                            } else if (0 <= y - 1 && map[x][y - 1] != '#' && map[x][y - 1] != '*') {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y - 1});
                                robot_i_points.add(1, new int[]{x, y - 1});
                            } else {
                                robot_i_points.add(1, new int[]{x, y});
                                robot_i_points.add(1, new int[]{x, y + 1});
                                robot_i_points.add(1, new int[]{x, y + 1});
                            }
                        }
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
        for (Integer idx : idxs) {
            List<int[]> sub_path = points.remove(0);
            sub_path.forEach(p -> robots[idx].path.addFirst(p));
        }
    }

    private void collision_avoidance2() {
        /**
         * 移动方向垂直 后一个机器人停一步
         * 移动方向相对 其中一个绕一步
         * 移动方向相同但后一个先动 后一个停一步
         * 移动的撞上stay的 我也stay
         * 下一个点上是否有机器人(机器人没路径)
         */
        int[] up = {-1, 0};
        int[] down = {1, 0};
        int[] left = {0, -1};
        int[] right = {0, 1};
        int[] stay = {0, 0};
        List<List<int[]>> points = new ArrayList<>();
        List<Integer> idxs = new ArrayList<>();
        for (int i = 0; i < robot_num; i++) {
            if (robots[i].path.isEmpty() || !fake_path.equals(Arrays.toString(robots[i].path.peek()))) {
                continue;
            }
            List<int[]> p = new ArrayList<>();
            p.add(robots[i].path.pollFirst());
            p.add(robots[i].path.pollFirst());
            points.add(p);
            idxs.add(i);
        }
        int length = points.size();
        for (int i = 1; i < length; i++) {
            for (int j = 0; j < i; j++) {
                //后机器人
                List<int[]> a_points = points.get(i);
                int[] a_now = a_points.get(0);
                int[] a_next = a_points.get(1);
                int[] a_dir = new int[]{a_next[0] - a_now[0], a_next[1] - a_now[1]};
                //前机器人
                List<int[]> b_points = points.get(i);
                int[] b_now = b_points.get(0);
                int[] b_next = b_points.get(1);
                int[] b_dir = new int[]{b_next[0] - b_now[0], b_next[1] - b_now[1]};
                if (Arrays.equals(a_dir, stay) && Arrays.equals(a_next, b_next)) {
                    //a静止且b下一步将要到达
                    b_points.add(0, b_now);
                } else if (Arrays.equals(b_dir, stay) && Arrays.equals(b_next, a_next)) {
                    //b静止且a下一步将要到达
                    a_points.add(0, a_now);
                } else if (!Arrays.equals(a_dir, stay) && !Arrays.equals(b_dir, stay)) {
                    //两个都有动作
                    int multi = a_dir[0] * b_dir[0] + a_dir[1] * b_dir[1];
                    if (multi == 0 && (Arrays.equals(a_next, b_next) || Arrays.equals(b_next, a_now))) {
                        //移动方向垂直 且先走的机器人走到后走机器人身上 前一个机器人停一步
                        b_points.add(0, b_now);
                    }
                    if (multi == -1 && Arrays.equals(a_next, b_next)) {
                        //移动方向相对 其中一个绕一步
                        if (Arrays.equals(a_dir, left) || Arrays.equals(a_dir, right)) {

                        }
                    }
                }
            }
        }
        for (Integer idx : idxs) {
            List<int[]> sub_path = points.remove(0);
            sub_path.forEach(p -> robots[idx].path.addFirst(p));
        }
    }

    private void boatPlan(int zhen, StringBuilder msg) {
        for (int i = 0; i < boat_num; i++) {
            if (boat[i].pos == -1 && boat[i].status == 1) {
                //空闲轮船
                int max = -1;
                int max_idx = 0;
                for (int j = 0; j < berth_num; j++) {
                    if (!berth[j].free) {
                        continue;
                    }
                    if (berth[j].goods_num > max) {
                        max = berth[j].goods_num;
                        max_idx = j;
                    }
                }
                System.out.printf("ship %d %d" + System.lineSeparator(), i, max_idx);
                berth[max_idx].free = false;
            } else if (boat[i].pos != -1 && boat[i].status == 1) {
                //轮船已经到达泊口
                if (boat[i].arrived_time < 0) {
                    boat[i].arrived_time = zhen;
                }
                int berth_id = boat[i].pos;
                int time = zhen - boat[i].arrived_time;
                int load_num = time * berth[berth_id].loading_speed;
                int num = berth[berth_id].goods_num;
                if (load_num >= num) {
                    //货物装完，轮船走
                    //余量较多，去别的泊位
                    boat[i].remain_capacity -= num;
                    if (boat[i].remain_capacity > 20) {
                        int max = -1;
                        int max_idx = -1;
                        for (int j = 0; j < berth_num; j++) {
                            if (!berth[j].free) {
                                continue;
                            }
                            //挑选泊位
                            int load_time = Math.min(berth[j].goods_num, boat[i].remain_capacity) / berth[j].loading_speed;
                            if (zhen + 500 + berth[j].transport_time + load_time >= 15000) {
                                //回不到虚拟点，放弃选择该泊位
                                continue;
                            }
                            if (berth[j].goods_num > max) {
                                max = berth[j].goods_num;
                                max_idx = j;
                            }
                            if (max > boat[i].remain_capacity) {
                                break;
                            }
                        }
                        if (max_idx != -1) {
                            //下一轮时间足够返回
                            System.out.printf("ship %d %d" + System.lineSeparator(), i, max_idx);
                            berth[max_idx].free = false;
                        } else {
                            //下一轮时间不够返回
                            System.out.printf("go %d" + System.lineSeparator(), i);
                        }
                    } else {
                        //余量较少，去虚拟点
                        System.out.printf("go %d" + System.lineSeparator(), i);
                    }
                    berth[berth_id].goods_num = 0;
                    berth[berth_id].free = true;
                    boat[i].arrived_time = -1;
                } else if (load_num > boat[i].remain_capacity) {
                    //轮船满，轮船走
                    System.out.printf("go %d" + System.lineSeparator(), i);
                    berth[berth_id].goods_num = num - boat[i].remain_capacity;
                    berth[berth_id].free = true;
                    boat[i].arrived_time = -1;
                    //重置轮船余量
                    boat[i].remain_capacity = boat_capacity;
                }
            }
            //行驶中的轮船不做处理
        }
        //测试
//        if (zhen == 5001) {
//            for (int i = 0; i < 10; i++) {
//                msg.append("berth:" + i + "  loading_speed:" + berth[i].loading_speed + " goods_num" + berth[i].goods_num + "  transport_time" + berth[i].transport_time + "\n");
//            }
//            for (int i = 0; i < 5; i++) {
//                msg.append("boat:" + i + "  pos" + boat[i].pos + "\n");
//            }
//            msg.append(boat_capacity + "\n");
//            throw new RuntimeException(msg.toString());
//        }
    }


    public static void main(String[] args) {
        Main mainInstance = new Main();
        mainInstance.init();

        int[] berth_idx = new int[5];
        Arrays.fill(semaphore, true);

        for (int zhen = 1; zhen <= 15000; zhen++) {
            //更新货物left time
            if (!mainInstance.gds.isEmpty()) {
                for (Good gd : mainInstance.gds) {
                    if (!mainInstance.gds_map[gd.x][gd.y]) {
                        continue;
                    }
                    gd.left_time--;
                    if (gd.left_time < 50) {
                        mainInstance.gds_map[gd.x][gd.y] = false;
                    }
                }
            }
            int id = mainInstance.input();
            //机器人调度
            for (int i = 0; i < robot_num; i++) {
                if (!mainInstance.robots[i].path.isEmpty() && !Arrays.toString(mainInstance.robots[i].path.peek()).equals(fake_path)) {
                    mainInstance.getRobotsCmd(mainInstance.robots[i], i, mainInstance.msg);
                } else {
                    if (!semaphore[i]) {
                        continue;
                    }
                    semaphore[i] = false;
                    int idx = i;
                    int z = zhen;
                    //创建子进程，处理路径规划问题
                    mainInstance.executor.execute(() -> {
                        if (z < 200) {
                            mainInstance.robot2Berth(mainInstance.robots[idx]);
                        } else {
                            mainInstance.robotPlan(mainInstance.robots[idx]);
                        }
                        semaphore[idx] = true;
                    });
                }
            }

            //轮船调度
            if (zhen >= 5000) {
                mainInstance.boatPlan(zhen, mainInstance.msg);
            }
//            try {
//                Thread.sleep(9);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
            System.out.println("OK");
            System.out.flush();
            //测试：
//            if (zhen == 14000) {
//                for (int i = 0; i < 5; i++) {
//                    mainInstance.msg.append("boat");
//                    mainInstance.msg.append(i);
//                    mainInstance.msg.append(":");
//                    mainInstance.msg.append("  num:" + mainInstance.boat[i].num);
//                    mainInstance.msg.append("  pos:" + mainInstance.boat[i].pos);
//                    mainInstance.msg.append("  status:" + mainInstance.boat[i].status + "\n");
//                }
//                throw new RuntimeException(mainInstance.msg.toString());
//            }
        }
    }

    class Robot {
        int x, y, goods;
        int status;
        int mbx, mby;
        boolean arrived;

        Deque<int[]> path;
        Deque<int[]> back_path;

        Deque<Character> path_mark;

        public Robot() {
            this.path = new LinkedList<>();
            this.back_path = new LinkedList<>();
            this.path_mark = new LinkedList<>();
        }

        public Robot(int startX, int startY) {
            this.x = startX;
            this.y = startY;
            this.path = new LinkedList<>();
            this.back_path = new LinkedList<>();
            this.path_mark = new LinkedList<>();
        }
    }

    class Berth {
        int x;
        int y;
        int transport_time;
        int loading_speed;
        int goods_num;

        boolean free;


        public Berth() {
        }

        public Berth(int x, int y, int transport_time, int loading_speed) {
            this.x = x;
            this.y = y;
            this.transport_time = transport_time;
            this.loading_speed = loading_speed;
            this.goods_num = 0;
            this.free = true;
        }
    }

    class Boat {
        int num;
        /**
         * 表示目标泊位，如果目标泊位是虚拟点，则为-1
         */
        int pos;
        /**
         * <p> 0 表示移动(运输)中
         * <p> 1 表示正常运行状态(即装货状态或运输完成状态)
         * <p> 2 表示泊位外等待状态
         */
        int status;

        int arrived_time;
        int remain_capacity;

        public Boat() {
            this.num = 0;
            this.arrived_time = -1;
            this.remain_capacity = boat_capacity;
        }
    }

    class Good {
        int x;
        int y;
        int val;
        int left_time;

        public Good() {
        }

        public Good(int x, int y, int val) {
            this.x = x;
            this.y = y;
            this.val = val;
            this.left_time = 1000;
        }
    }
}
