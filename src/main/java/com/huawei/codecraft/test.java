package com.huawei.codecraft;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.ReentrantLock;

public class test {
    //信号里
    //private static Semaphore semaphore = new Semaphore(5);//允许个数，相当于放了5把锁
    private static volatile boolean[] s = new boolean[5];
    private static ReentrantLock lock = new ReentrantLock(true);

    private static List<Ro> a = new ArrayList<>();

    static class Ro {
        int x;
        int y;

        Ro(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }


    public static void main(String[] args) {
        Arrays.fill(s, true);
        for (int i = 0; i < 100; i++) {
            a.add(new Ro(i, i));
        }
        while (true) {
//            for (Ro ro : a) {
//                ro.x--;
//                ro.y++;
//                System.out.println(ro.x);
//                System.out.println(ro.y);
//            }
            for (int i = 0; i < 5; i++) {
                int idx = i;
                if (!s[idx]) {
                    continue;
                }
                s[idx] = false;
                new Thread(new Runnable() {
                    public void run() {
                        method(idx);
                    }
                }).start();
            }
            if (lock.tryLock()) {
                System.out.println("主线程get lock了");
                lock.unlock();
            }
        }
    }

    private static void method(int idx) {
        System.out.println(idx);
        lock.lock();
        try {
            System.out.println("ThreadName=" + Thread.currentThread().getName() + "我过来了");
//            for (Ro r : a) {
//                int temp= r.x;
//                r.x = r.y;
//                r.y = temp;
//                System.out.println("ThreadName=" + Thread.currentThread().getName() + "  " + r.toString());
//            }
            Thread.sleep(500);
            System.out.println("ThreadName=" + Thread.currentThread().getName() + "出去了");
            s[idx] = true;//释放信号
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        } finally {
            lock.unlock();
        }
    }
}
