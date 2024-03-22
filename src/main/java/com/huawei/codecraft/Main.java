/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 */

package com.huawei.codecraft;

import sun.nio.cs.ext.MS874;

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
    private static final int step_size = 8;
    private static final int N = 210;
    private static final String fake_path = "[-1, -1]";
    private static final int[] fake_goal = {-1, -1};

    private int money, boat_capacity, id;
    private String[] ch = new String[N];
    /**
     * ‘.’ ： 空地 <p>
     * ‘*’ ： 海洋 <p>
     * ‘#’ ： 障碍 <p>
     * ‘A’ ： 机器人起始位置，总共 10 个 <p>
     * ‘B’ ： 大小为 4*4，表示泊位的位置,泊位标号在后泊位处初始化
     */
    private char[][] map = new char[n][n];
    private boolean[][] gds_map = new boolean[n][n];
    private Good[][] gds_entity = new Good[n][n];
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
    private ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(1);

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
            Good gd = new Good(x, y, val);
            gds.add(gd);
            gds_map[x][y] = true;
            gds_entity[x][y] = gd;
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
        Random rand = new Random();
        int min_dist = 200;
        int rx = robot.x, ry = robot.y;
        Deque<int[]> path = new LinkedList<>();
        Deque<int[]> goal_path = new LinkedList<>();
        //todo 遍历货物寻找最优路径(优化：最有价值)
        Good best = null;
        for (Good gd : gds) {
            if (!gds_map[gd.x][gd.y]) {
                continue;
            }
            if (Arrays.equals(robot.goal, fake_goal) && min_dist < 50) {
                Astar astar = new Astar(map, gds_map);
                path = astar.aStarSearchGoodEarlyStop(new int[]{rx, ry}, new int[]{best.x, best.y}, msg);
//                Bfs bfs = new Bfs(map, gds_map);
//                robot.path = bfs.bfsSearchGood(new int[]{rx, ry}, new int[]{best.x, best.y});
                //路径不空且时间足够到达，更新货物生存时间,并记录返回路径
                if (!path.isEmpty() && best.left_time > path.size()) {
                    int[] end = path.peekLast();
                    gds_map[end[0]][end[1]] = false;
                    robot.goal = end;
                    robot.goal_dist = path.size() - 1;
                    for (int i = 0; i < step_size + rand.nextInt(5); i++) {
                        if (!path.isEmpty())
                            robot.path.addLast(path.pollFirst());
                    }
                    markPath(robot);
                    return;
                }
            }
            int dist = Math.abs(gd.x - rx) + Math.abs(gd.y - ry);
            if (dist < min_dist && dist < robot.goal_dist) {
                min_dist = dist;
                best = gd;
            }
        }
        Astar astar = new Astar(map, gds_map);
        Bfs bfs = new Bfs(map, gds_map);
        /**
         *有目标、找到best
         *无目标、找到best
         *有目标、无best
         *无目标、无best
         */
        if (!Arrays.equals(robot.goal, fake_goal) && best != null) {
//            path = astar.aStarSearchGood(new int[]{rx, ry}, new int[]{best.x, best.y}, msg);
//            goal_path = astar.aStarSearchGood(new int[]{rx, ry}, new int[]{robot.goal[0], robot.goal[1]}, msg);
            path = bfs.bfsSearchGood(new int[]{rx, ry}, new int[]{best.x, best.y});
            goal_path = bfs.bfsSearchGood(new int[]{rx, ry}, new int[]{robot.goal[0], robot.goal[1]});
            if (!path.isEmpty() && path.size() < goal_path.size()) {
                //更新目标
                int[] end = path.peekLast();
                gds_map[robot.goal[0]][robot.goal[1]] = true;
                gds_map[end[0]][end[1]] = false;
                robot.goal = end;
                goal_path = path;
            }
            robot.goal_dist = goal_path.size() - 1;
            for (int i = 0; i < step_size + rand.nextInt(5); i++) {
                if (!goal_path.isEmpty())
                    robot.path.addLast(goal_path.pollFirst());
            }
            markPath(robot);
        } else if (Arrays.equals(robot.goal, fake_goal) && best != null) {
            path = astar.aStarSearchGoodEarlyStop(new int[]{rx, ry}, new int[]{best.x, best.y}, msg);
            if (!path.isEmpty()) {
                //更新目标
                int[] end = path.peekLast();
                gds_map[end[0]][end[1]] = false;
                robot.goal = end;
                robot.goal_dist = path.size() - 1;
                for (int i = 0; i < step_size + rand.nextInt(5); i++) {
                    if (!path.isEmpty())
                        robot.path.addLast(path.pollFirst());
                }
                markPath(robot);
            }
        } else if (!Arrays.equals(robot.goal, fake_goal) && best == null) {
            //goal_path = astar.aStarSearchGood(new int[]{rx, ry}, new int[]{robot.goal[0], robot.goal[1]}, msg);
            goal_path = bfs.bfsSearchGood(new int[]{rx, ry}, new int[]{robot.goal[0], robot.goal[1]});
            robot.goal_dist = goal_path.size() - 1;
            for (int i = 0; i < step_size + rand.nextInt(5); i++) {
                if (!goal_path.isEmpty())
                    robot.path.addLast(goal_path.pollFirst());
            }
            markPath(robot);
        }
    }

    /**
     * 拿到货物的机器人到港口
     *
     * @param robot 机器人
     */
    private void robot2Berth(Robot robot) {
        if (inBerth(robot) > -1) {
            return;
        }
        Deque<int[]> path = new LinkedList<>();
        int rx = robot.x, ry = robot.y;
        if (Arrays.equals(robot.goal, fake_goal)) {
            Random rand = new Random();
            //遍历港口寻找最优路径
            Queue<int[]> dist_que = new PriorityQueue<int[]>(Comparator.comparingInt(e -> e[1]));
            for (int j = 0; j < berth_num; j++) {
                int dist = Math.abs(berth[j].x - rx) + Math.abs(berth[j].y - ry);
                dist_que.add(new int[]{j, dist});
            }
            while (!dist_que.isEmpty()) {
                int[] pair = dist_que.poll();
                int num = pair[0];
//            Astar astar = new Astar(map);
//            path = astar.aStarSearchBerth(new int[]{rx, ry}, new int[]{berth[num].x + rand.nextInt(4), berth[num].y + rand.nextInt(4)});
                Bfs bfs = new Bfs(map);
                path = bfs.bfsSearchBerthEarlyStop(new int[]{rx, ry}, new int[]{berth[num].x + rand.nextInt(4), berth[num].y + rand.nextInt(4)});
                if (!path.isEmpty()) {
                    break;
                }
            }
        } else {
            Bfs bfs = new Bfs(map);
            path = bfs.bfsSearchBerthEarlyStop(new int[]{rx, ry}, new int[]{robot.goal[0], robot.goal[1]});
        }
        if (!path.isEmpty()) {
            robot.goal = path.peekLast();
            for (int i = 0; i < 5; i++) {
                if (!path.isEmpty())
                    robot.path.addLast(path.pollFirst());
            }
        }
        markPath(robot);
    }

    private int inBerth(Robot robot) {
        int rx = robot.x;
        int ry = robot.y;
        for (int i = 0; i < berth_num; i++) {
            int x = berth[i].x;
            int y = berth[i].y;
            if (rx >= x && rx <= x + 3 && ry >= y && ry <= y + 3) {
                return i;
            }
        }
        return -1;
    }

    private int inBerth(int rx, int ry) {
        for (int i = 0; i < berth_num; i++) {
            int x = berth[i].x;
            int y = berth[i].y;
            if (rx >= x && rx <= x + 3 && ry >= y && ry <= y + 3) {
                return i;
            }
        }
        return -1;
    }

    /**
     * 对机器人路径进行标记，使得路径规划时可以绕过已规划的路径，降低机器人碰撞概率
     *
     * @param robot
     */
    private void markPath(Robot robot) {
        if (robot.path.isEmpty()) {
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
                if (robot.goal_dist != 300) {
                    robot.goal_dist--;
                }
            }
        }
        if (path.size() == 1) {
            now = path.poll();
            if (robot.goods == 0 && robot.goal_dist == 0) { //无货取货
                System.out.printf("get %d" + System.lineSeparator(), idx);
                gds_map[now[0]][now[1]] = false;
                robot.goal = fake_goal;
                robot.goal_dist = 300;
                robot.carry = gds_entity[now[0]][now[1]];
            } else { //有货卸货
                int id = inBerth(now[0], now[1]);
                if (id > -1 && robot.goods == 1) {
                    System.out.printf("pull %d" + System.lineSeparator(), idx);
                    berth[id].goods_num++;
                    berth[id].gds.add(robot.carry);
                    berth[id].total_val += robot.carry.val;
                    robot.goal = fake_goal;
                    robot.carry = null;
                }
            }
        }
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
    private void robotPlan(Robot robot, StringBuilder msg) {
        if (robot.goods == 0) {
            //取货路径规划
            robot2Good(robot, msg);
        } else {
            //送货路径规划
            robot2Berth(robot);
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
                            if (zhen + 700 + berth[j].transport_time + load_time >= 15000) {
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

    private void boatPlanValue(int zhen, StringBuilder msg) {
        for (int i = 0; i < boat_num; i++) {
            if (boat[i].pos == -1 && boat[i].status == 1) {
                //空闲轮船
                int max_val = -1;
                int max_idx = 0;
                for (int j = 0; j < berth_num; j++) {
                    int val = berth[j].goods_num >= boat_capacity ? getFirstNVal(berth[j], boat_capacity) : berth[j].total_val;
                    if (val > max_val) {
                        max_val = val;
                        max_idx = j;
                    }
                }
                System.out.printf("ship %d %d" + System.lineSeparator(), i, max_idx);
                //更新港口及轮船信息
                if (berth[max_idx].goods_num >= boat_capacity) {
                    boat[i].num = boat_capacity;
                    boat[i].remain_capacity = 0;
                    berth[max_idx].goods_num -= boat_capacity;
                    berth[max_idx].total_val -= max_val;
                    removeFirstNGoods(berth[max_idx],boat_capacity);
                } else {
                    boat[i].num = berth[max_idx].goods_num;
                    boat[i].remain_capacity -= berth[max_idx].goods_num;
                    berth[max_idx].goods_num = 0;
                    berth[max_idx].total_val = 0;
                    removeFirstNGoods(berth[max_idx],berth[max_idx].goods_num);
                }
            } else if (boat[i].pos != -1 && boat[i].status == 1) {
                //轮船已经到达泊口
                if (boat[i].arrived_time < 0) {
                    boat[i].arrived_time = zhen;
                }
                int berth_id = boat[i].pos;
                int time = zhen - boat[i].arrived_time;
                int load_num = time * berth[berth_id].loading_speed;
                if (load_num >= boat[i].num) {
                    //货物装完，轮船走
                    //余量较多，去别的泊位
                    if (boat[i].remain_capacity > 15) {
                        int max_val = -1;
                        int max_idx = -1;
                        for (int j = 0; j < berth_num; j++) {
                            //挑选泊位
                            int load_time = Math.min(berth[j].goods_num, boat[i].remain_capacity) / berth[j].loading_speed;
                            if (zhen + 700 + berth[j].transport_time + load_time >= 15000) {
                                //回不到虚拟点，放弃选择该泊位
                                continue;
                            }
                            int val = berth[j].goods_num >= boat[i].remain_capacity ? getFirstNVal(berth[j], boat[i].remain_capacity) : berth[j].total_val;
                            if (val > max_val) {
                                max_val = val;
                                max_idx = j;
                            }
                        }
                        if (max_idx != -1) {
                            //下一轮时间足够返回
                            System.out.printf("ship %d %d" + System.lineSeparator(), i, max_idx);
                            //更新港口及轮船信息
                            if (berth[max_idx].goods_num >= boat[i].remain_capacity) {
                                boat[i].num = boat[i].remain_capacity;
                                boat[i].remain_capacity = 0;
                                berth[max_idx].goods_num -= boat[i].remain_capacity;
                                berth[max_idx].total_val -= max_val;
                                removeFirstNGoods(berth[max_idx],boat[i].remain_capacity);
                            } else {
                                boat[i].num = berth[max_idx].goods_num;
                                boat[i].remain_capacity -= berth[max_idx].goods_num;
                                berth[max_idx].goods_num = 0;
                                berth[max_idx].total_val = 0;
                                removeFirstNGoods(berth[max_idx],berth[max_idx].goods_num);
                            }
                        } else {
                            //下一轮时间不够返回
                            System.out.printf("go %d" + System.lineSeparator(), i);
                            boat[i].arrived_time = -1;
                            boat[i].remain_capacity = boat_capacity;
                        }
                    } else {
                        //余量较少，去虚拟点
                        System.out.printf("go %d" + System.lineSeparator(), i);
                        boat[i].arrived_time = -1;
                        boat[i].remain_capacity = boat_capacity;
                    }
                    boat[i].arrived_time = -1;
                }
            }
            //行驶中的轮船不做处理
        }
    }

    private int getFirstNVal(Berth berth, int n) {
        if (berth.goods_num < n) {
            return -1;
        }
        int value = 0;
        Iterator<Good> iterator = berth.gds.iterator();
        for (int i = 0; i < n; i++) {
            Good next = iterator.next();
            value += next.val;
        }
        return value;
    }

    private void removeFirstNGoods(Berth berth, int n) {
        if (berth.goods_num < n) {
            return;
        }
        for (int i = 0; i < n; i++) {
            berth.gds.poll();
        }
    }

    public static void main(String[] args) {
        Main mainInstance = new Main();
        mainInstance.init();

        Arrays.fill(semaphore, true);

        for (int zhen = 1; zhen <= 15000; zhen++) {
            long start1 = System.currentTimeMillis();
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
            //机器人调度,多线程
//            for (int i = 0; i < robot_num; i++) {
//                if (!mainInstance.robots[i].path.isEmpty()) {
//                    mainInstance.getRobotsCmd(mainInstance.robots[i], i, mainInstance.msg);
//                } else {
//                    if (!semaphore[i]) {
//                        continue;
//                    }
//                    semaphore[i] = false;
//                    int idx = i;
//                    int z = zhen;
//                    //创建子进程，处理路径规划问题
//                    mainInstance.executor.execute(() -> {
//                        if (z < 150) {
//                            mainInstance.robot2Berth(mainInstance.robots[idx]);
//                        } else {
//                            mainInstance.robotPlan(mainInstance.robots[idx], mainInstance.msg);
//                        }
//                        semaphore[idx] = true;
//                    });
//                }
//            }
            //机器人调度,主线程算5个robot，子线程算5个robot
//            for (int i = 0; i < robot_num; i++) {
//                if (!mainInstance.robots[i].path.isEmpty()) {
//                    mainInstance.getRobotsCmd(mainInstance.robots[i], i, mainInstance.msg);
//                } else {
//                    if (i < 5) {
//                        if (zhen < 150) {
//                            mainInstance.robot2Berth(mainInstance.robots[i]);
//                        } else {
//                            mainInstance.robotPlan(mainInstance.robots[i], mainInstance.msg);
//                        }
//                    } else {
//                        if (!semaphore[i]) {
//                            continue;
//                        }
//                        semaphore[i] = false;
//                        int idx = i;
//                        int z = zhen;
//                        //创建子进程，处理路径规划问题
//                        mainInstance.executor.execute(() -> {
//                            if (z < 150) {
//                                mainInstance.robot2Berth(mainInstance.robots[idx]);
//                            } else {
//                                mainInstance.robotPlan(mainInstance.robots[idx], mainInstance.msg);
//                            }
//                            semaphore[idx] = true;
//                        });
//                    }
//                }
//            }
            //机器人调度,单线程
            for (int i = 0; i < robot_num; i++) {
                if (mainInstance.robots[i].path.isEmpty()) {
                    if (zhen < 100) {
                        mainInstance.robot2Berth(mainInstance.robots[i]);
                    } else {
                        mainInstance.robotPlan(mainInstance.robots[i], mainInstance.msg);
                    }
                }
                if (!mainInstance.robots[i].path.isEmpty()) {
                    mainInstance.getRobotsCmd(mainInstance.robots[i], i, mainInstance.msg);
                }
            }
            //轮船调度
            if (zhen >= 3000) {
                mainInstance.boatPlan(zhen, mainInstance.msg);
            }
//            long end1 = System.currentTimeMillis();
//            long rest_time = 14 - (end1 - start1);
//            try {
//                Thread.sleep(10);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
            //主进程等待解决跳帧问题
//            long start2 = System.currentTimeMillis();
//            while (true) {
//                long now = System.currentTimeMillis();
//                if (now - start2 > rest_time){
//                    break;
//                }
//            }
            System.out.println("OK");
            System.out.flush();
            //测试：
//            if (zhen == 14500) {
//                for (int i = 0; i < 10; i++) {
//                    mainInstance.msg.append("robot:" + i);
//                    mainInstance.msg.append("  inBerth:" + mainInstance.inBerth(mainInstance.robots[i]));
//                    mainInstance.msg.append("  goods:" + mainInstance.robots[i].goods);
//                    mainInstance.msg.append("  goal:" + mainInstance.robots[i].goal);
//                    mainInstance.msg.append("  goal_dist:" + mainInstance.robots[i].goal_dist);
//                    mainInstance.msg.append("  path:");
//                    for (int[] ints : mainInstance.robots[i].path) {
//                        mainInstance.msg.append(" " + Arrays.toString(ints));
//                    }
//                    mainInstance.msg.append("\n");
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
        Deque<int[]> path = new LinkedList<>();
        Deque<Character> path_mark = new LinkedList<>();
        int[] goal = new int[]{-1, -1};
        int goal_dist = 300;
        Good carry = null;

        public Robot() {

        }

        public Robot(int startX, int startY) {
            this.x = startX;
            this.y = startY;
            this.path = new LinkedList<>();
            this.path_mark = new LinkedList<>();
        }
    }

    class Berth {
        int x;
        int y;
        int transport_time;
        int loading_speed;
        int goods_num = 0;
        Queue<Good> gds = new LinkedList<>();
        int total_val = 0;
        boolean free = true;


        public Berth() {
        }

        public Berth(int x, int y, int transport_time, int loading_speed) {
            this.x = x;
            this.y = y;
            this.transport_time = transport_time;
            this.loading_speed = loading_speed;
        }
    }

    class Boat {
        /**
         * 每次到泊口要运输的数量
         */
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
        int arrived_time = -1;
        int remain_capacity = boat_capacity;

        public Boat() {
            this.num = 0;
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
