package com.neocoretechs.robocore;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Class to manage thread resources throughout the application. Singleton. Fixed thread pool.
 * Attached a run completion method that decrements a countdown latch until all threads up to executionLimit
 * have completed, or use the standard Futures array for finer grained control.
 * @author jg
 *
 */
public class SynchronizedFixedThreadPoolManager {
	int threadNum = 0;
    private static Map<String, FactoryThreadsLimit> executor = new ConcurrentHashMap<String, FactoryThreadsLimit>();
	public static volatile SynchronizedFixedThreadPoolManager threadPoolManager = null;
	private SynchronizedFixedThreadPoolManager() { }

	public static SynchronizedFixedThreadPoolManager getInstance() {
		if( threadPoolManager == null )
			synchronized(SynchronizedFixedThreadPoolManager.class) {
				if(threadPoolManager == null) {
					threadPoolManager = new SynchronizedFixedThreadPoolManager();
				}
			}
		return threadPoolManager;
	}
	
	/**
	 * Create an array of Executors that manage a cached thread pool for
	 * reading topics. One thread pool per topic to notify listeners of data ready
	 * @param threadGroupNames The topics for which thread groups are established
	 */
	public static void init(int maxThreads, int executionLimit, String[] threadGroupNames) {
		for(String tgn : threadGroupNames) {
			//executor.put(tgn, getInstance(maxExecution, executionLimit).new ExtendedExecutor(maxExecution, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit)));
			DaemonThreadFactory dtf = (getInstance().new DaemonThreadFactory(tgn));
			ExecutorService tpx = (getInstance().new ExtendedExecutor(maxThreads, executionLimit, new LinkedBlockingQueue<Runnable>(), dtf));
			executor.put(tgn,getInstance().new FactoryThreadsLimit(tgn, dtf, tpx, maxThreads, executionLimit));
			((ExtendedExecutor)tpx).prestartAllCoreThreads();
		}
	}
	
	public void init(int maxThreads, int executionLimit, String group) {
		FactoryThreadsLimit ftl = executor.get(group);
		if( ftl != null ) {
			ftl.exs.shutdownNow();
			executor.remove(ftl.group);
		}
		DaemonThreadFactory dtf = new DaemonThreadFactory(group);
		ExecutorService tpx = new ExtendedExecutor(maxThreads, executionLimit, new LinkedBlockingQueue<Runnable>(), dtf);
		executor.put(group, new FactoryThreadsLimit(group, dtf, tpx, maxThreads, executionLimit));
		((ExtendedExecutor)tpx).prestartAllCoreThreads();
	}
	
	public void init(int maxThreads, int executionLimit) {
		FactoryThreadsLimit ftl = executor.get("SYSTEMSYNC");
		if( ftl != null ) {
			ftl.exs.shutdownNow();
			executor.remove(ftl.group);
		}
		DaemonThreadFactory dtf = new DaemonThreadFactory("SYSTEMSYNC");
		ExecutorService tpx = new ExtendedExecutor(maxThreads, executionLimit, new LinkedBlockingQueue<Runnable>(), dtf);
		executor.put("SYSTEMSYNC", new FactoryThreadsLimit("SYSTEMSYNC", dtf, tpx, maxThreads, executionLimit));
		((ExtendedExecutor)tpx).prestartAllCoreThreads();
	}
	
	public static void resetLatch(int count) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
		ftl.maxExecution = count;
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public static void resetLatch(int count, String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
		ftl.maxExecution = count;
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public static void waitForGroupToFinish(String group) throws InterruptedException {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
		((ExtendedExecutor)exe).getLatch().await();
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public static void waitForGroupToFinish() throws InterruptedException {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
		((ExtendedExecutor)exe).getLatch().await();
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public static BlockingQueue<Runnable> getQueue(String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
		return ((ExtendedExecutor)exe).getQueue();
		//return ((ThreadPoolExecutor)executor.get(group)).getQueue();
	}
	
	public static BlockingQueue<Runnable> getQueue() {
		//return ((ThreadPoolExecutor)executor.get("SYSTEMSYNC")).getQueue();
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
		return ((ExtendedExecutor)exe).getQueue();
	}

	public static void waitGroup(String group) {
		try {
			FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
			ExecutorService w = ftl.exs;
			//ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait();
			}
		} catch (InterruptedException e) {
		}
	}
	
	public static void waitGroup(String group, long millis) {
		try {
			FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
			ExecutorService w = ftl.exs;
			//ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait(millis);
			}
		} catch (InterruptedException e) {
		}
	}
	
	public static void waitForCompletion(Future<?>[] futures) {
	    	//System.out.println("waitForCompletion on:"+futures.length);
	        int size = futures.length;
	        try {
	            for (int j = 0; j < size; j++) {
	                futures[j].get();
	            }
	        } catch (ExecutionException ex) {
	            ex.printStackTrace();
	        } catch (InterruptedException e) {
	            e.printStackTrace();
	        }
	}
	
	public static void notifyGroup(String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService w = ftl.exs;
		//ExecutorService w = executor.get(group);
		synchronized(w) {
			w.notifyAll();
		}
	}
	
	public static void spin(Runnable r, ThreadGroup group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group.getName()));
		ExecutorService exe = ftl.exs;
	    /*executor.get(group.getName())*/exe.execute(r);
	}
	
	public static void spin(Runnable r, String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
	    /*executor.get(group)*/exe.execute(r);
	}
	
	public static void spin(Runnable r) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
	    /*executor.get("SYSTEMSYNC")*/exe.execute(r);
	}
	
    public static Future<?> submit(Runnable r) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
        return ftl.exs.submit(r);
    }
    
	public static Future<?> submit(Runnable r, String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
	    return exe.submit(r);
	}
    
	public static void shutdown() {
		//Collection<ExecutorService> ex = executor.values();
		Collection<FactoryThreadsLimit> ex = executor.values();
		for(/*ExecutorService*/FactoryThreadsLimit e : ex) {
			List<Runnable> spun = e.exs.shutdownNow();
			for(Runnable rs : spun) {
				System.out.println("Marked for Termination:"+rs.toString()+" "+e.toString());
			}
		}
	}
	
	class DaemonThreadFactory implements ThreadFactory {
		ThreadGroup threadGroup;
	
		public DaemonThreadFactory(String threadGroupName) {
			threadGroup = new ThreadGroup(threadGroupName);
		}	
		public ThreadGroup getThreadGroup() { return threadGroup; }		
	    public Thread newThread(Runnable r) {
	        Thread thread = new Thread(threadGroup, r, threadGroup.getName()+(++threadNum));
	        //thread.setDaemon(true);
	        return thread;
	    }
	}
	
	class ExtendedExecutor extends ThreadPoolExecutor {
		public CountDownLatch latch;
		public ExtendedExecutor(int maxThreads, int executionLimit, BlockingQueue<Runnable> threadQueue, DaemonThreadFactory dtf) {
			super(maxThreads, executionLimit, Long.MAX_VALUE, TimeUnit.DAYS, threadQueue, dtf );
			latch = new CountDownLatch(executionLimit);
		}
		public CountDownLatch getLatch() { return latch; }
		
		@Override
		protected void afterExecute(Runnable r, Throwable t) {
			super.afterExecute(r, t);
		  // regardless of errors, etc, count down the latch
			latch.countDown();
		}
		
	}
	
	class FactoryThreadsLimit {
		public DaemonThreadFactory dtf;
		public ExecutorService exs;
		public int totalThreads;
		public int maxExecution;
		public String group;
		public FactoryThreadsLimit(String group, DaemonThreadFactory dtf, ExecutorService exs, int totalThreads, int maxExecution ) {
			this.group = group;
			this.dtf = dtf;
			this.exs = exs;
			this.totalThreads = totalThreads;
			this.maxExecution = maxExecution;
		}
		@Override
		public boolean equals(Object o) {
			return group.equals(((FactoryThreadsLimit)o).group);
		}
		@Override
		public int hashCode() {
			return group.hashCode();
		}
	}
}
